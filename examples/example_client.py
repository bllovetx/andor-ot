from sipyco.pc_rpc import Client
import numpy as np
import pylab as pl

def get_photon_nb_from_str(data: str) -> int:
    pixel_nb = int(np.floor(len(data)/2))
    temp_count = 0
    for i in range(pixel_nb):
        temp_count = temp_count + ord(data[2*i])*256 + ord(data[2*i+1])
    return temp_count

def show_img_from_str(data: str, width: int, height: int) -> pl.matplotlib.image.Axesimage: 
    pixel_nb = int(np.floor(len(data)/2))
    pixels = np.zeros(pixel_nb)
    for i in range(pixel_nb):
        pixels[i] = ord(data[2*i])*256 + ord(data[2*i+1])
    return pl.imshow(
        np.reshape(pixels, (width, height))
    )

def main():
    emccd = Client("192.168.50.187", 3249, "emccd")
    emccd.start_acquisition_with_watcher()
    # pulse trigger
    # stop acquisition
    emccd.stop_data_watcher()
    # get data from pipeline until empty
    photon_counts = []
    tempdata = emccd.get_data_from_pipeline()
    while tempdata:
        photon_counts.append(
            get_photon_nb_from_str(tempdata)
        )
        tempdata = emccd.get_data_from_pipeline()
    emccd.close_rpc()

if __name__ == "__main__":
    main()