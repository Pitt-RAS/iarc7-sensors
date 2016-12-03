# iarc7\_sensors

Note: The altimeter node depends on the `libi2c-dev` package, which can be installed by running

    sudo apt-get install libi2c-dev

By default, users don't have permission to access the linux i2c driver.  You can change that by running

    sudo usermod -a -G i2c <user>

If you don't do this, you'll have to run the altimeter node as root.
