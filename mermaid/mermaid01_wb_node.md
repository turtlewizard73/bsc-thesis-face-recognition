ROS nodeok
<!-- #c24c48
#df8395
#edbdd4
#fff7ff
#c7c6df
#7d9dbe
#067891 -->
>

https://mycolor.space/?hex=%23159AB7&sub=1
classDef node fill:#1e5067,stroke:#333,stroke-width:0px,color:#fff;
    style main fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;
        style videocapture fill:#159ab7,stroke:#333,stroke-width:2px;
        style cvbridge fill:#159ab7,stroke:#333,stroke-width:2px;
        style pub fill:#159ab7,stroke:#333,stroke-width:2px;





webcam node basic io
```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#1e5067', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
flowchart LR
    wc("Videocamera live feed")
    wc_node("Webcam node")
    ip(["image_publisher.py"])
    pub("Publisher")
    img_topic["/image/webcam/raw"]
    subgraph wc_io [Webcam node I/O]
    style wc_io fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;
    direction LR
    wc --> wc_node_sub
        subgraph wc_node_sub [ ]
        direction TB
            wc_node --> ip
        end
    wc_node_sub --> pub
    pub --> img_topic

    end
```


```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#1e5067', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
flowchart LR
    vc(cv2.VideoCapture)
    vc2([".set()"])
    rp("rospy.Publisher")
    topic("/image/webcam/raw")
    cvb("cv_bridge.CvBridge")
    cvb2(["bridge.cv2_to_imgmsg()"])

    subgraph wc_main [Webcam node]
    style wc_main fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;
    direction LR
        subgraph videocapture [ ]
            vc --> vc2
        end
        videocapture == frame ==> cvbridge
        subgraph cvbridge [ ]
            cvb -->cvb2
        end
        cvbridge == sensor_msgs.msg.Image ==> pub
        subgraph pub [ ]
        rp --> topic
        end
    end
```

face_recognition core node
```mermaid

flowchart TD
    a ==> b
```

database ndoe
```mermaid

%%{ init: { 'flowchart': { 'curve': 'stepBefore' } } }%%
graph LR
a-->s
```
