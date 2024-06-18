# Total Energy Controller

The total energy control system approach (TECS) differs from the successive loop closure controller in its control of airspeed and altitude.
It does this by calculating the total energy of the system, both potential and kinetic and controlling it to the desired energy.
A more in depth treatment of of TECS see Section 6.2 in the UAVbook.

The TECS controller has been included in ROSplane not only to provide control of the aircraft but to also demonstrate how to easily modify the controller (see [Software Architecture](./controller-software-architecture.md) for more details).
