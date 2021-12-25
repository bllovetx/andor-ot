import andor as ad
from sipyco.pc_rpc import simple_server_loop
import logging

def main():
    # config log
    logging.basicConfig(
        level=logging.INFO, filename="./logs/serverlog",
        format='%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    # start server
    # TODO: change ip to current server's ip, NOTE don't use ::1(localhost)
    simple_server_loop({"emccd": ad.by_serial_number()}, "192.168.50.187", 3249)


if __name__ == "__main__":
    main()