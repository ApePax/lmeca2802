Commandes pour run le code C en julia

Compilation du code C:
gcc -shared -fPIC -o libmyfunc.so myfunc.c

Compilation du code Julia:
include("main.jl")

