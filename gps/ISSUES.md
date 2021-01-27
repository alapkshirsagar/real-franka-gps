Issues with GPS on the KUKA LWR 4+
======

- The PR2 plugin has a parameter-free update() method, whereas the KUKA
  controllers have times. This causes compilation problems due to pure virtual
  functions.
