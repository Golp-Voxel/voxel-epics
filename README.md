# voxel-epics
Epics Repository for Voxel Experiment at IPFN-IST.

<!-- TOC start (generated with https://github.com/derlin/bitdowntoc) -->

- [How to install EPICS](#how-to-install-epics)
   * [Add the asyn package](#add-the-asyn-package)
   * [Install StreamDevice (by Dirk Zimoch, PSI)](#install-streamdevice-by-dirk-zimoch-psi)
- [How to Create a IOC](#how-to-create-a-ioc)
- [Refereces](#refereces)

<!-- TOC end -->

## How to install EPICS

First, make sure your machine is updated by running the following commands:

```
sudo apt-get upgrade
```

```
sudo apt-get update
```



```
sudo addgroup epics
```

```
sudo usermod -aG epics $USER$
```

```
mkdir $HOME/EPICS
```

```
cd $HOME/EPICS
```

```
git clone --recursive https://github.com/epics-base/epics-base.git
```
```
cd epics-base
```
```
make
```
This action may take a long time to finish.


After compiling you should put the path into `$HOME/.profile` or into `$HOME/.bashrc` by adding the following to either one of those files:
```
export EPICS_BASE=${HOME}/EPICS/epics-base
export EPICS_HOST_ARCH=$(${EPICS_BASE}/startup/EpicsHostArch)
export PATH=${EPICS_BASE}/bin/${EPICS_HOST_ARCH}:${PATH}
```

### Add the asyn package

```
cd $HOME/EPICS
mkdir support
cd support
git clone https://github.com/epics-modules/asyn.git
cd asyn
```

Edit $HOME/EPICS/support/asyn/configure/RELEASE and set EPICS_BASE like:
```
EPICS_BASE=${HOME}/EPICS/epics-base
```
Now, run
```
make
```
If the build fails due to implicit declaration of xdr_* functions it is likely that asyn should build against libtirpc. To do so, you can uncomment # TIRPC=YES in configure/CONFIG_SITE of asyn, such that it states:
```
# Some linux systems moved RPC related symbols to libtirpc
# To enable linking against this library, uncomment the following line
TIRPC=YES
```

### Install StreamDevice (by Dirk Zimoch, PSI)

```
cd $HOME/EPICS/support
git clone https://github.com/paulscherrerinstitute/StreamDevice.git
cd StreamDevice/
rm GNUmakefile
```

Edit `$HOME/EPICS/support/StreamDevice/configure/RELEASE` to specify the install location of EPICS base and of additional software modules, for example:
```
EPICS_BASE=${HOME}/EPICS/epics-base
SUPPORT=${HOME}/EPICS/support
ASYN=$(SUPPORT)/asyn
```

## How to Create a IOC 

To create a IOC of EPICS, frist create a folder:
```
mkdir voxelRpi
cd voxelRpi
```
```
makeBaseApp.pl -t <type> <app>
```
The `<type>` can be support, ioc, example, caClient or caServer. The `<app>` is teh name of the app that is going to be created. By running the following comand, 

```
makeBaseApp.pl -i -t <type> <ioc>
```
it will create a IOC with with the `<ioc>` name given by the user.

After edit the file `configure/RELEASE` of the generated aplication and add the following lines:
```
ASYN=/usr/local/epics/modules/asyn
STREAM=/usr/local/epics/modules/stream
```


## Refereces

* [EPICS Documentation](https://docs.epics-controls.org/en/latest/getting-started/installation-linux.html#installation-on-linux-macos)

* [isttok-epics](https://github.com/bernardocarvalho/isttok-epics/tree/master)
* [esther-epics](https://github.com/ipfn-hpl/esther-epics)