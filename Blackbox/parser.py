import argparse

# Create the parser
parser = argparse.ArgumentParser()

# Add an argument
parser.add_argument("-f", "--flag", help="Turn on or off", action="store_true")

# Parse the arguments
args = parser.parse_args()

# Use the argument
if args.flag:
    print("Option is turned on")
else:
    print("Option is turned off")