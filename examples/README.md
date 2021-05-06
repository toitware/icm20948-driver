# Example

This example already comes with a `package.lock` file that references the
package it is included in. If you want to run this example outside the
package directory do the following:

* Copy the `accel_gyro.toit` file into a folder of your choice.
* Optionally run `toit pkg init --app` in this folder. This sets the root of
  the program with an empty `package.lock` file.
* Synchronize the registry: `toit pkg sync`.
* Install the package: `toit pkg install icm20948`.

## Troubleshooting
If installing the package didn't work, you might be missing a registry. See
https://docs.toit.io/package/pkgguide/ for instructions on adding a registry.
