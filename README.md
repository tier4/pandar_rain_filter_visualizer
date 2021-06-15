# Pandar Rain Filter Visualizer

Command-line Interface for Pandar rain filter visualizing tool. The tool expects file path containing the rain labels and range images stored in the following structure:

```
└── file_path
    ├─particle_labels
    │   ├─train
    │   │ ├─first
    │   │ │ ├─0.png
    │   │ │ ├─1.png
    │   │ │ ├─ ...
    │   │ └─last
    │   │   ├─0.png
    │   │   ├─1.png
    │   │   ├─ ...
    │   └─val
    │     ├─first
    │     │ ├─0.png
    │     │ ├─1.png
    │     └─last
    │       ├─0.png
    │       ├─1.png
    │       ├─ ...
    └─range_images
        ├─train
        │ ├─first_depth
        │ │ ├─0.png
        │ │ ├─1.png
        │ │ ├─ ...
        │ ├─first_intensity
        │ │ ├─0.png
        │ │ ├─1.png
        │ │ ├─ ...
        │ ├─first_return_type
        │ │ ├─0.png
        │ │ ├─1.png
        │ │ ├─ ...
        │ ├─last_depth
        │ │ ├─0.png
        │ │ ├─1.png
        │ │ ├─ ...
        │ ├─last_intensity
        │ │ ├─0.png
        │ │ ├─1.png
        │ │ ├─ ...
        │ └─last_return_type
        │   ├─0.png
        │   ├─1.png
        │   ├─ ...
        └─val
          ├─first_depth
          │ ├─0.png
          │ ├─1.png
          │ ├─ ...
          ├─first_intensity
          │ ├─0.png
          │ ├─1.png
          │ ├─ ...
          ├─first_return_type
          │ ├─0.png
          │ ├─1.png
          │ ├─ ...
          ├─last_depth
          │ ├─0.png
          │ ├─1.png
          │ ├─ ...
          ├─last_intensity
          │ ├─0.png
          │ ├─1.png
          │ ├─ ...
          └─last_return_type
            ├─0.png
            ├─1.png
            ├─ ...

## Usage

```sh
rosrun pandar_rain_filter_visualizer pandar_rain_filter_visualizer _file_path:=[data file path]
```

# Parameters and options

 |Param|Default Value|Description|
 |---|---|---|
 |`file_path`|None|Path of folder containing labels and range images|



