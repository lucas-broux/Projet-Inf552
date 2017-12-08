################################################################################
# The purpose of this script is to analyse the output of the performance       #
# evaluation program.                                                          #
# We produce graph plots and data analysis.                                    #
# Python version: 3.*                                                          #
#                                                                              #
# @authors: Romain LOISEAU & Lucas BROUX                                       #
################################################################################

# Plotting library.
import matplotlib.pyplot as plt
# Library for litteral evaluation of strings.
import ast
# Library for paths management.
import os.path

# Open file.
file = open('performance_report_python.txt', 'r')

# Open dict of performance data.
performance_data = dict()
performance_data_units = dict()

# Loop over lines.
for line in file:
    # Get info as dict.
    line_info = ast.literal_eval(line.split("\n")[0])

    # Extract infos from dict.
    input_nb = int(line_info['INPUT'])
    info = line_info['INFO']
    performance = float(line_info['PERF'])
    performance_unit = line_info['UNIT']

    # Add data in dict.
    if (performance_data.get(info) == None):
        performance_data[info] = [performance]
        performance_data_units[info] = performance_unit
    else:
        performance_data[info].append(performance)


# Show performance data for first info.
for key, value in performance_data.items():
    # Generate histogram plot.
    plt.hist(value)
    # Generate title.
    plt.title(key)
    # Generate legends.
    unit = performance_data_units[key]
    if (unit == "seconds"):
        plt.xlabel("Time (seconds)")
    elif (unit == ""):
        plt.xlabel(key + " (units)")
    else:
        plt.xlabel(key + " (" + unit + ")")
    plt.ylabel("Frequency")
    # Save and show plot.
    plt.savefig(os.path.join('performance_plots', key + '.png'), bbox_inches='tight')
    plt.show()
