face recognition node basic io
```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#1e5067', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
flowchart TB
    topic1["/image/webcam/raw"]
    sub1("Subscriber 1")
    topic2["/face/queried_encodings"]
    sub2("Subscriber 2")
    wc_node("Face recognition node")
    ip(["scripts/face_recognition_core.py"])
    pub1("Publisher 1")
    topic3["/image/detected_faces"]
    pub2("Publisher 2")
    topic4["/face/encodings"]
    service("Service 1")
    server[get_db]


    subgraph wc_i [Face recognition node I/O]
    direction LR
        style wc_i fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;
        subgraph sub [Subscribers]
            direction TB
            topic1 --> sub1
            topic2 --> sub2
        end
        sub1 --> fr_node_g
        sub2 --> fr_node_g
        subgraph fr_node_g [ ]
            wc_node --> ip
        end
        fr_node_g --> pub1
        fr_node_g --> pub2
        subgraph pub [Publishers]
            direction LR
            pub1 --> topic3
            pub2 --> topic4
        end
        fr_node_g --> service
        service --> fr_node_g
        subgraph serv [Service]
        service --> server
        end
    end
```
face recognition data interface input
```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#1e5067', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
flowchart LR
    topic1["/image/webcam/raw"]
    sub1("Subscriber 1")
    topic2["/face/queried_encodings"]
    sub2("Subscriber 2")
    server["get_db"]
    service("Service 1")

    wc_node("Face recognition node")

    di("DataInterface")
    di2([".msg2enc()"])

    cvb("CvBridge")
    cvb2([".imgmsg_to_cv2()"])
    subgraph wc_i [ ]
        direction LR
        style wc_i fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;
        fr["frame: numpy.ndarray(numpy.uint)"]
        subgraph sub [Inputs: subscribers, services]
            direction TB
            topic1 --> sub1
            topic2 --> sub2
            server --> service
        end
        sub1 -- Image.msg --> cvb
        sub2 -- Encoding.mg --> di
        service --  DataFrame.srv.Response --> di
        
        subgraph di_node [DataInterface]
            direction TB
            cvb --> cvb2
            di -->di2
        end
        cvb2 -->|"frame"| wc_node
        di2 -->|"encodings \n names \n uids"| wc_node
        enc["encodings: list(numpy.darray(numpy.float64)) \n names: list(string) \n uids: list(string)"]

    end
```

face recognition data interface output
```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#1e5067', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
flowchart TB
    fr_node("Face recognition node")
    pub1("Publisher 1")
    topic3["/image/detected_faces"]
    pub2("Publisher 2")
    topic4["/face/encodings"]
    service("Service 1")
    server[get_db]
    di("DataInterface")
    di2([".enc2msg()"])

    cvb("CvBridge")
    cvb2([".cv2_to_imgmsg()"])

    subgraph wc_o [ ]
    direction LR
        style wc_o fill:#acdbdf,stroke:#333,stroke-width:0px,color:#1e5067;

        fr_node -->|"encodings \n names \n uids"| di
        fr_node -->|frame| cvb
        fr_node -->|Dataframe.srv| service
        subgraph di_node [DataInterface]
            direction TB
            cvb --> cvb2
            di -->di2
        end
        cvb2 -- Image.msg --> pub1
        di2 -- Encoding.msg --> pub2
        subgraph pub [Outputs: publishers, service requests]
            direction LR
            pub1 --> topic3
            pub2 --> topic4
            service --> server
        end
    end
```




























<!-- 
```mermaid
%%{init: {'theme': 'base','themeVariables': {'darkMode':'false','background':'#fff', 'primaryColor':'#1e5067', 'primaryBorderColor':'#000000', 'primaryTextColor': '#fff', 'secondaryColor':'#6BB6C7', 'tertiaryColor':'#c1faff', 'tertiaryBorderColor':'#000000', 'clusterBkg':'#159ab7'}}}%%
classDiagram
    class Face_publisher{
        self.datainterface
        self.imageProcessor
        self.bridge
        self.tolerance
        self.image_pub
        self.encoding_pub
        self.image_sub
        self.update_cache_sub
    }
``` -->