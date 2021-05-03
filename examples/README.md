# Example

This example already comes with a `package.lock` file that references the
package it is included in. If you want to run this example outside the
package directory do the following:

* Copy the `icm20948.toit` file into a folder of your choice.
* In this folder run `toit pkg init --app`.
* Synchronize the registry: `toit pkg sync`.
* Install the package: `toit pkg install icm20948`.

## Troubleshooting
If installing the package didn't work, you might be missing a registry. See
https://docs.toit.io/package/pkgguide/ for instructions on adding a registry.
