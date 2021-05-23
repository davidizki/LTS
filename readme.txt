Main file:
main (the solver is called from here; the figures are generated)

Settings file:
LTS_settings (the numerical solver options are set up)

Problem definition file:
LTS (the OCP -including ICs, constants, fixed models, look-up tables, constraints definition-
and the initial guess are DEFINED)

System dynamics file:
LTS_dynamics (for a given control and state, the state function (ODEs) and the constraints are EVALUATED)
