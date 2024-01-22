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
To just run the simulator without GDB, omit the last line: :code:`-S -gdb tcp::9822`.
All other options are mandatory, the simulator will likely not work without them.

.. code-block:: shell

  ./build/qemu-system-arm \
	-machine max78000 -smp 3 \
	-serial stdio -serial file:/tmp/comp1.out -serial file:/tmp/comp2.out \
	-kernel <path to ap.elf> -bios <path to comp.elf> \
	 -S -gdb tcp::9822

Then, you can connect GDB to the simulator by running :code:`arm-none-eabi-gdb`, then typing:

.. code-block:: shell

  target remote :9822
  add-symbol-file <path to ap.elf>

Any :code:`printf`'s from the AP get echoed to :code:`stdout`, where you run QEMU. The :code:`printf`'s
from the components get sent to :code:`/tmp/comp1.out` and :code:`/tmp/comp2.out`, if you run as
configured above. NOTE THAT THE ORDER OF THESE OPTIONS MATTERS. The first :code:`-serial` option always
corresponds to the AP, and the next ones always correspond to components 1 and 2 respectively.
