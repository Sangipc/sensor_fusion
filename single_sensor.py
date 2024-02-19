import asyncio
import multiprocessing
import measurement

from scanner import Scanner
def run_process(fn, *args):
    "Spawn a process and run the function `fn`"
    process = multiprocessing.Process(target=fn, args=args)
    process.start()
    return process

Ts = 0.01  # sampling rate
N = 2  # number of IMUs

def main():
    manager = multiprocessing.Manager()
    sensor_dict = manager.dict()
    devices = asyncio.run(Scanner.scan_and_filter_xsens_dot())
    devices = [(device.name, device.address) for device in devices]
    print(devices)
    processes = []
    for id in range(N):
        sensor_dict[id] = manager.Queue()
        process = run_process(measurement.always_read_imu, id, sensor_dict, devices)
        processes.append(process)

    process = run_process(measurement.always_process_data, sensor_dict)
    processes.append(process)

    for process in processes:
        process.join()

if __name__ == "__main__":
    main()
