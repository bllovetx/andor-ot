import andor as ad
from sipyco.pc_rpc import simple_server_loop
import logging

def main():
    logging.basicConfig(
        level=logging.INFO, filename="./logs/serverlog",
        format='%(asctime)s.%(msec)03d %(levelname)s %(module)s - %(funcName)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    simple_server_loop({"emccd": ad.by_serial_number()}, "192.168.50.187", 3249)


if __name__ == "__main__":
    main()