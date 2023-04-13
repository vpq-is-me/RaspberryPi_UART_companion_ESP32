set(srcs "main.c"
        "board.c"
        "loop.c"
        "crc8.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
