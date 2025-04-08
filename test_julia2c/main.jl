# main.jl

println("Calling C code from Julia...")

const lib = "./libmyfunc.so"  # Adjust the path if needed

# Call void hello()
ccall((:hello, lib), Cvoid, ())

# Call int add(int, int)
a = 10
b = 20
result = ccall((:add, lib), Cint, (Cint, Cint), a, b)

println("Result from C: $a + $b = $result")
