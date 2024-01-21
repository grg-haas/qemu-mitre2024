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
To just run the simulator without GDB, omit :code:`-S -gdb tcp::9822`.

.. code-block:: shell

  ./build/qemu-system-arm -machine max78000 -nographic -kernel <path to ap.elf> -S -gdb tcp::9822

Then, you can connect GDB to the simulator by running :code:`arm-none-eabi-gdb`, then typing:

.. code-block:: shell

  target remote :9822
  add-symbol-file <path to ap.elf>
