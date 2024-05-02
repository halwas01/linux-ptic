// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Texas Instruments TLA2528 ADC
 *
 * Copyright (C) 2020-2021 Rodolfo Giometti <giometti@enneenne.com>
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define TLA2528_OP_READ_REG		0x10
#define TLA2528_OP_WRITE_REG		0x08

#define TLA2528_GENERAL_CFG		0x01
#define TLA2528_GENERAL_CFG_CNVST	BIT(3)
#define TLA2528_DATA_CFG		0x02
#define TLA2528_DATA_CFG_APPEND_STATUS	BIT(4)
#define TLA2528_DATA_CFG_FIX_PAT	BIT(7)
#define TLA2528_PIN_CFG			0x05
#define TLA2528_SEQUENCE_CFG		0x10
#define TLA2528_CHANNEL_SEL		0x11

struct tla2528_st {
	struct i2c_client *client;
	struct regulator *ref;

	u8 last_read_channel;
};

static s32 i2c_smbus_read_sample(const struct i2c_client *client)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].buf = (u8 *) &data;
	msg[0].len = 2;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		return ret;

	return le16_to_cpu(data[1] | (data[0] << 8));
}

static s32 i2c_smbus_write_reg(const struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg[1];
	u8 cmd[3] = {TLA2528_OP_WRITE_REG, reg, val};

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = cmd;
	msg[0].len = 3;

	return i2c_transfer(client->adapter, msg, 1);
}

static int tla2528_read(struct tla2528_st *st, u8 channel, int *val)
{
	struct i2c_client *client = st->client;
	int ret;

	if (channel != st->last_read_channel) {
		ret = i2c_smbus_write_reg(st->client,
					TLA2528_CHANNEL_SEL, channel);
		if (ret < 0)
			return ret;

		st->last_read_channel = channel;
	}

	/* Read ADC data (2 bytes) */
	ret = i2c_smbus_read_sample(client);
	if (ret < 0)  {
		dev_err(&client->dev, "i2c_master_recv failed\n");
		return ret;
	}
	*val = ret >> 4;

	return 0;
}

static int tla2528_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tla2528_st *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		ret = tla2528_read(st, chan->channel, val);
		mutex_unlock(&indio_dev->mlock);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->ref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = 12;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

#define TLA2528_CHAN(_chan, _name) { \
	.type = IIO_VOLTAGE,					\
	.channel = (_chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = _name,				\
	.indexed = 1,						\
}

static const struct iio_chan_spec tla2528_channel[] = {
	TLA2528_CHAN(0, "AIN0"),
	TLA2528_CHAN(1, "AIN1"),
	TLA2528_CHAN(2, "AIN2"),
	TLA2528_CHAN(3, "AIN3"),
	TLA2528_CHAN(4, "AIN4"),
	TLA2528_CHAN(5, "AIN5"),
	TLA2528_CHAN(6, "AIN6"),
	TLA2528_CHAN(7, "AIN7"),
};

static const struct iio_info tla2528_info = {
	.read_raw = tla2528_read_raw,
};

static int tla2528_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct tla2528_st *st;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_WRITE_BYTE))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	st->client = client;

	indio_dev->name = id->name;
	indio_dev->info = &tla2528_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = tla2528_channel;
	indio_dev->num_channels = ARRAY_SIZE(tla2528_channel);

	st->ref = devm_regulator_get(&client->dev, "vref");
	if (IS_ERR(st->ref))
		return PTR_ERR(st->ref);

	ret = regulator_enable(st->ref);
	if (ret < 0)
		return ret;

	/* Set all inputs as analog */
	ret = i2c_smbus_write_reg(st->client, TLA2528_PIN_CFG, 0x00);
	if (ret < 0)
		goto err_regulator_disable;

	ret = i2c_smbus_write_reg(st->client, TLA2528_DATA_CFG,
				  TLA2528_DATA_CFG_APPEND_STATUS);
	if (ret < 0)
		goto err_regulator_disable;

	/* Set manual mode */
	ret = i2c_smbus_write_reg(st->client, TLA2528_SEQUENCE_CFG, 0x00);
	if (ret < 0)
		goto err_regulator_disable;

	/* Init private data */
	st->last_read_channel = ~0;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err_regulator_disable;

	return 0;

err_regulator_disable:
	regulator_disable(st->ref);

	return ret;
}

static int tla2528_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct tla2528_st *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	regulator_disable(st->ref);

	return 0;
}

static const struct i2c_device_id tla2528_id[] = {
	{ "tla2528", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tla2528_id);

static const struct of_device_id tla2528_of_match[] = {
	{ .compatible = "ti,tla2528", },
	{ }
};
MODULE_DEVICE_TABLE(of, tla2528_of_match);

static struct i2c_driver tla2528_driver = {
	.driver = {
		.name = "tla2528",
		.of_match_table = tla2528_of_match,
	},
	.probe = tla2528_probe,
	.remove = tla2528_remove,
	.id_table = tla2528_id,
};
module_i2c_driver(tla2528_driver);

MODULE_AUTHOR("Rodolfo Giometti <giometti@enneenne.com>");
MODULE_DESCRIPTION("Texas Instruments TLA2528 ADC driver");
MODULE_LICENSE("GPL v2");
