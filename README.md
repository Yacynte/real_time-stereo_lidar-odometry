# stereo_lidar_recorder
# Realtime Stereo Visual Odometry
# StereoLiDARRecorder Build Instructions

## 1. Overview
This guide provides step-by-step instructions to build the **StereoLiDARRecorder** project for:
- **Raspberry Pi 4** using Docker for cross-compilation
- **PC (x86_64)** for native development

## 2. Prerequisites
Ensure you have the following installed on your **PC**:
- **Docker** (for Raspberry Pi 4 cross-compilation)
- **CMake** (>=3.10)
- **GCC/G++** (for native builds)
- **OpenCV & PCL** (for both target platforms)

## 3. Building for PC (x86_64)

### **Step 1: Install Dependencies**
Run the following command to install required dependencies:

```bash
sudo apt update && sudo apt install -y cmake g++ libopencv-dev libpcl-dev
```

### **Step 2: Build the Project**
Navigate to the project root and create a build directory:

```bash
cd ~/stereo_lidar_recorder
mkdir -p build_pc && cd build_pc
```

Run CMake and compile:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=toolchain_pc.cmake
make -j$(nproc)
```

### **Step 3: Run the Executable**

```bash
./StereoLiDARRecorder
```

---

## 4. Cross-Compiling for Raspberry Pi 4 via Docker

### **Step 1: Build the Docker Image**
Inside the `rpi-cross-compile` folder, create a **Dockerfile** if it doesn’t exist:

```dockerfile
FROM ubuntu:20.04
RUN apt update && apt install -y cmake g++ aarch64-linux-gnu-g++ libopencv-dev libpcl-dev
WORKDIR /workspace
```

Now, build the Docker image:

```bash
cd ~/stereo_lidar_recorder/rpi-cross-compile
docker build -t rpi-cross-compile .
```

### **Step 2: Run the Docker Container**
Run the container and mount the workspace:

```bash
docker run -it --rm -v ~/stereo_lidar_recorder:/workspace rpi-cross-compile
```

### **Step 3: Build the Project**
Inside the container:

```bash
cd /workspace
mkdir -p build_rpi && cd build_rpi
cmake .. -DCMAKE_TOOLCHAIN_FILE=/workspace/toolchain_rpi.cmake -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### **Step 4: Transfer and Run on Raspberry Pi**
Copy the binary to the Raspberry Pi:

```bash
scp StereoLiDARRecorder pi@raspberrypi:/home/pi/
```

On the Raspberry Pi, run:

```bash
chmod +x StereoLiDARRecorder
./StereoLiDARRecorder
```

---

## 5. Troubleshooting
### **1. CMake Cannot Find OpenCV or PCL**
Make sure OpenCV and PCL are installed. If needed, specify their paths in `CMakeLists.txt`:

```cmake
set(OpenCV_DIR "/usr/lib/cmake/opencv4")
set(PCL_DIR "/usr/lib/cmake/pcl")
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
```

### **2. Docker Image Not Found**
Rebuild the Docker image:

```bash
cd ~/stereo_lidar_recorder/rpi-cross-compile
docker build -t rpi-cross-compile .
```

### **3. Binary Not Executable on Raspberry Pi**
Ensure the binary is built for `aarch64`:

```bash
file StereoLiDARRecorder
```

If incorrect, check your `toolchain_rpi.cmake` file for errors.

---

## 6. Notes
- Adjust the toolchain files (`toolchain_pc.cmake` and `toolchain_rpi.cmake`) as needed.
- Use `make clean` before rebuilding if errors occur.

---

## 7. License
This project is licensed under the MIT License.

---

## 8. Author
Batchaya Yacynte

