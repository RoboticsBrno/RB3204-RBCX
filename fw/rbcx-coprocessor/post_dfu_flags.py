Import("env")

env.Replace(UPLOADERFLAGS=[
        "-d", "vid:pid,0483:df11",
        "-a", "0", "-R", "-D"
    ])
