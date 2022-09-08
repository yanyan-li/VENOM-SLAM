## Dockfile

Since Venom makes use of Pangolin for visualization, we link VNC Viewer to the basic environment. 

#### 1. VNC-based Visualization 

VNC Viewer could be found [here](https://www.realvnc.com/en/connect/download/viewer/). 

#### 2. docker 

**1.1 docker image**

```
docker build -t IndoorSLAM/venom .
```

**1.2 docker container**

To use local VS Code, we share local documents to the docker container by 5900 Port.

```
# local address for the documents: /home/your_name/Documents/VENOM/
docker run -itd -v /home/your_name/Documents/VENOM/:/home/VENOM/ -p 5900:5900 -e PASSWORD=password InddorSLAM/venom
```

**1.3 Open VNC Viewer** 

```
# input the following address on your VNC Viewer
127.0.0.1:5900 
```

![vcn](../images/vcn.png)
