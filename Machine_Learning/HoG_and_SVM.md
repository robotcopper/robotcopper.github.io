---
title: HoG & SVM
time: 2024-09-22
---

# [HoG & SVM](https://github.com/robotcopper/object_detection_HOG-SVM_pipeline)
<br>
In the field of object detection in computer vision, the use of robust and well-structured pipelines is essential to ensure reliable results, especially in resource-constrained environments. The HOG-SVM object detection pipeline I developed integrates the Histogram of Oriented Gradients (HOG) method as a feature extractor and Support Vector Machines (SVM) as the classifier. This system is designed to offer an efficient and accessible solution for prototyping, training, and validating object detection models using the Dlib library.<br>

This pipeline provides a powerful alternative for those looking to implement detection systems without the need for a GPU, while still achieving solid results from limited datasets.

## Table of Contents

1. [Motivation](#motivation)
2. [What is HOG+SVM?](#what-is-hogsvm)
3. [Pipeline Structure](#pipeline-structure)
4. [Requirements](#requirements)
5. [Usage Instructions](#usage-instructions)  
   5.1. [Resizing Images](#1-resizing-images)  
   5.2. [Labeling Images](#2-labeling-images)  
   5.3. [Training the Model](#3-training-the-model)  
   5.4. [Validating the Model](#4-validating-the-model)  
   5.5. [Testing with Pre-trained Model](#5-testing-with-pre-trained-model)
6. [Advantages and Disadvantages](#advantages-and-disadvantages)
7. [License](#license)

<br>

## Motivation

This project was motivated by the desire to return to a simpler neural network architecture, specifically using a classifier, to address a basic object identification problem.<br>
In the provided example, the goal is to detect a single class—such as a red ball—where deploying a computationally expensive model like a Convolutional Neural Network (CNN), nowadays very popular, requiring a GPU would be disproportionate. <br>
At the same time, traditional image processing techniques (e.g., dilation, erosion, edge detection) tend to be too sensitive to environmental variations, such as lighting changes, and thus lack generalization.<br>

The HOG-SVM pipeline offers a lightweight yet effective solution for simple object detection tasks, striking a balance between computational efficiency and robustness in varying conditions.
<br>

### What is HOG+SVM?

**HOG (Histogram of Oriented Gradients)** is a feature extraction method that describes the structure and appearance of an object by analyzing gradient orientations in localized portions of an image. It’s widely used in object detection, especially for its robustness to variations in lighting and pose.

**SVM (Support Vector Machine)** is a supervised machine learning algorithm that classifies data by finding the hyperplane that best separates the feature space into categories. 

In the context of object detection, HOG features are extracted from the images and fed into an SVM for training. The trained SVM can then be used to classify objects in new images.

For a visual understanding of the HOG+SVM process:
<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/HOG_SVM/HoG2SVM.png" style="background: transparent;" width="100%" >
</div>
<br>

## Pipeline Structure

```
object_detection_HOG-SVM_pipeline
├── assets
├── images
│   ├── resizer.py
│   ├── test
│   ├── train
│   └── validation
├── README.md
├── resources
│   ├── ball_recognition_exemple.svm
│   ├── test_label.xml
│   └── train_label.xml
└── scripts
    ├── detector4images.py
    ├── detector4webcam.py
    └── train.py
```
<br>

## Requirements

Before running the pipeline, ensure that you have the following Python libraries installed:

- **Dlib** (for object detection and training)
- **OpenCV** (for image processing and webcam access)
- **NumPy** (for numerical computations)

To install these dependencies, run the following command:

```bash
pip install dlib opencv-python numpy
```

Additionally, make sure you have a C++ compiler installed to build Dlib, as it requires compilation. On Ubuntu, you can install the necessary packages with:

```bash
sudo apt-get install build-essential cmake
sudo apt-get install libgtk-3-dev libboost-all-dev
```

Once these are installed, you should be ready to run the pipeline.
<br>

## Usage Instructions

### 1. Resizing Images
To begin, you must resize the training, test, and validation images based on your desired detection frequency. The resizing script adjusts the image resolution to achieve a balance between frequency and resolution, as shown in this graph:

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/HOG_SVM/Detection_Frequency_as_a_Function_o_ Resolution .png" style="background: transparent;" width="60%" >
</div>

 By default, the frequency is set to 10Hz for an image resolution of 288x216px.


Run the following command in the `images` directory:
```bash
python resizer.py
```

### 2. Labeling Images
To generate the XML label files for Dlib, you can use the [**Imglab**](https://solothought.com/imglab/) labeling tool. Once the labeling is done, save the files in Dlib's XML format. Place the generated label files in the `resources` folder. 

The sample label files for training and testing are already provided (`train_label.xml` and `test_label.xml`).

### 3. Training the Model
Before training, ensure that the paths to the images are correctly set in the XML label files. Once confirmed, execute the training script:

```bash
python train.py
```

This script uses Dlib to train an object detector based on the labeled data. You can customize the hyperparameters of the training process by referring to the Dlib documentation: [Dlib Object Detector Training Options](http://dlib.net/python/index.html#dlib_pybind11.simple_object_detector_training_options).

### 4. Validating the Model
To validate the trained model on test images, execute the following command:

```bash
python detector4images.py
```

The script will output the results of the model's performance on the test images.

### 5. Testing with Pre-trained Model
If you want to test the pipeline using a pre-trained model, a trained SVM model on 100 images is provided (`ball_recognition_exemple.svm`). You can test it on your webcam by running:

```bash
python detector4webcam.py
```

An example of the webcam output:
<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/HOG_SVM/output_webcam_detector.gif" style="background: transparent;" width="25%" >
</div>
<br>

## Advantages and Disadvantages

| **Advantages** | **Disadvantages** |
|----------------|-------------------|
| Works without a GPU | Requires a minimum image size |
| Small dataset is sufficient for good results (5 images here) | Minimum object size (400px²) required for detection |
| High frequency achievable with small image sizes | Small objects in the image may not be detectable due to lack of detail |

By following this pipeline, you can quickly train and test object detection models using HOG+SVM, even with limited computational resources.
<br>

## License

This project is licensed under the [BSD 3-Clause License](https://github.com/robotcopper/object_detection_HOG-SVM_pipeline/blob/main/LICENSE).