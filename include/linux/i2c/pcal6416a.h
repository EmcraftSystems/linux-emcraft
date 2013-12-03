/* platform data for the PCAL6416A 16-bit I/O expander driver */

struct pcal6416a_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	uint16_t	invert;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	
	char		**names;
};
