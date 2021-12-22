from artiq.experiment import *
# from artiq.language.core import delay, kernel


class ExampleDeviceInEnvExperiment(EnvExperiment):
    """Test client in dash board"""
    def build(self):
        self.setattr_device("emccd")

    def run(self):
        # start acquisition
        self.emccd.start_acquisition_with_watcher()
        # pulse trigger
        # stop acquisition
        self.emccd.stop_data_watcher()
        # get data from pipeline until empty
        datas = []
        tempdata = self.emccd.get_data_from_pipeline()
        while tempdata:
            datas.append(tempdata)
            tempdata = self.emccd.get_data_from_pipeline()

