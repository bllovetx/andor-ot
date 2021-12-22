from sipyco.pc_rpc import Client

def main():
    emccd = Client("192.168.", 3249, "hello")
    emccd.start_acquisition_with_watcher()
    # pulse trigger
    # stop acquisition
    emccd.stop_data_watcher()
    # get data from pipeline until empty
    datas = []
    tempdata = emccd.get_data_from_pipeline()
    while tempdata:
        datas.append(tempdata)
        tempdata = emccd.get_data_from_pipeline()
    emccd.close_rpc()

if __name__ == "__main__":
    main()