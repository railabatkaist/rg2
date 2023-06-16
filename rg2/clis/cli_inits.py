import os


def _setup_cli():
    # create a file on the user's home path.
    os.makedirs(os.path.expanduser("~/.raisim"), exist_ok=True)

    # make activation.raisim file if it doesn't exist

    do_make = False
    if os.path.exists(os.path.expanduser("~/.raisim/activation.raisim")):
        print("RG2 :: Activation file already exists. Recreate?")
        if input("RG2 :: [y/n] : ") == "y":
            do_make = True
    else:
        do_make = True

    if do_make:
        print("RG2 :: Creating activation file...")
        print("RG2 :: Please enter your activation code.")

        activation_string = input("RG2 :: Activation code : ")

        with open(os.path.expanduser("~/.raisim/activation.raisim"), "w") as f:
            f.write(activation_string)

    # append ../linux/include to LD_LIBRARY_PATH
    # relative to this filepath

    libpath = os.path.join(os.path.dirname(__file__), "../linux/lib")
    libpath = os.path.abspath(libpath)

    if "LD_LIBRARY_PATH" not in os.environ:
        os.environ["LD_LIBRARY_PATH"] = libpath
    else:
        # if it exists, and it doesn't contain the path, append it
        if libpath not in os.environ["LD_LIBRARY_PATH"].split(":"):
            os.environ["LD_LIBRARY_PATH"] += f":{libpath}"

    print("RG2 :: LD_LIBRARY_PATH set to", os.environ["LD_LIBRARY_PATH"])
    # add that to .bashrc
    # if it doesn't exist, create it
    # if it does exist, append it

    with open(os.path.expanduser("~/.bashrc"), "a") as f:
        f.write(f"export LD_LIBRARY_PATH={os.environ['LD_LIBRARY_PATH']}\n")


def main():
    _setup_cli()
