.. _epit:

EPIT Sample
###########

Overview
********
A simple example which uses i.MX EPIT timer.

Building and Running
********************

This project starts the timer with 1 second interval and stops it after 5 seconds.
It can be built and executed on UDOO Neo board as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/epit
   :host-os: unix
   :board: udoo_neo_full
   :goals: run
   :compact:

Sample Output
=============

.. code-block:: console

    EPIT sample app is starting...
    counter started 0
    EPIT clock frequency is 1000
    counter set alarm 0
    timer tick
    timer tick
    timer tick
    timer tick
    timer tick
    counter stopped 0
