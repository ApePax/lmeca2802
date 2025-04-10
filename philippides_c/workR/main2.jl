# Set the LD_LIBRARY_PATH to include the ./build directory
"""
# Simple code to call the Robotran executable from julia
args = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
joined_args = join(string.(args), ",")  # Add ',' between numbers
println("Passing string: ", joined_args)
cd("build") do
    #run(`./exe_philippides_c $joined_args`)
    output = read(`./exe_philippides_c $joined_args`, String)
    results_line = split(output, "\n")[end-1]  # Assuming second last line is your result
    results = parse.(Float64, split(results_line))
    println("Results from C code: ", results)
end
"""
# Set the LD_LIBRARY_PATH to include the ./build directory if needed
# ENV["LD_LIBRARY_PATH"] = "./build:" * get(ENV, "LD_LIBRARY_PATH", "")

# Prepare input arguments
args = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
joined_args = join(string.(args), ",")  # Join with commas
println("Passing string: ", joined_args)

# Run the C executable and get results
results = cd("build") do
    output = read(`./exe_philippides_c $joined_args`, String)
    lines = split(output, "\n")
    result_line = last(lines)  # Assuming last line has the results
    parse.(Float64, split(result_line))
end

println("Results from C code: ", results)

