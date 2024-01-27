===========
QEMU README
===========

This is a QEMU fork for Batman's Kitchen, implementing simulation infrastructure for the Mitre 2024 eCTF.

Building
========

The :code:`--mitre-design` option to :code:`configure` is mandatory. It should point to the directory that
holds the :code:`application_processor` and :code:`msdk` directories.

.. code-block:: shell

  mkdir build
  cd build
  ../configure --enable-debug --disable-strip \
	--without-default-features --disable-werror \
	--enable-system --target-list="arm-softmmu" \
	--mitre-design=<path to mitre design>
  make -j$(nproc)

Running
=======

To run the simulator such that it waits for a GDB connection before starting, run the command below.
All options are mandatory, the simulator will likely not work without them.

.. code-block:: shell

  ./build/qemu-system-arm \
	-machine max78000,ap=<path to ap.elf>,comp1=<path to comp1.elf>,comp2=<path to comp2.elf>\
	-smp 3 -serial mon:stdio -serial file:/tmp/comp1.out -serial file:/tmp/comp2.out \
	-S -gdb tcp::9822

Any :code:`printf`'s from the AP get echoed to :code:`stdout`, where you run QEMU. The :code:`printf`'s
from the components get sent to :code:`/tmp/comp1.out` and :code:`/tmp/comp2.out`, if you run as
configured above. NOTE THAT THE ORDER OF THESE OPTIONS MATTERS. The first :code:`-serial` option always
corresponds to the AP, and the next ones always correspond to components 1 and 2 respectively.

Debugging
=========

You can debug the firmwares using :code:`printf`-style debugging as described above, or also using GDB
(highly highly recommended). To start QEMU with support for GDB, append the options :code:`-S -gdb tcp::9822`
to the QEMU command line above. Then, you can connect GDB to the simulator by running :code:`arm-none-eabi-gdb`,
then typing:

.. code-block:: shell

  target remote :9822
  add-symbol-file <path to ap.elf>
  add-symbol-file <path to comp.elf>

These commands can also be placed in a :code:`.gdbinit` file for convenience. One important note with GDB
debugging: both the AP firmware and the component firmware live in the same memory space. That is, for example,
there is a valid address :code:`0x10010200` in BOTH firmwares. If you add both symbol files to GDB as shown
above, GDB will most likely get REALLY confused since it only sees memory addresses -- it doesn't have
enough semantic understanding to know when :code:`0x10010200` refers to an address in the AP versus an address
in a component. As such, you should probably only have one of these symbol files loaded at a time.
