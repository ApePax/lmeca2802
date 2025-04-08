# Set the LD_LIBRARY_PATH to include the ./build directory

# Simple code to call the Robotran executable from julia
args = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
joined_args = join(string.(args), ",")  # Add ',' between numbers
println("Passing string: ", joined_args)
cd("build") do
    run(`./exe_philippides_c $joined_args`)
end
