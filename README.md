# Apple size estimation using photogrammetry-derived 3D point clouds

## Introduction
This project is a matlab implementation for apple size estimation in 3D point clouds. Four different size estimation methods are implemented: largest segment, least squares, MSAC and template matching. This code was used in [1] to compare the performance of the mentioned methods by using the [PFuji-Size dataset](http://www.grap.udl.cat/en/publications/PFuji-Size_dataset.html) (not publicly available yet). Find more information in:
* [In-field apple size estimation using photogrammetry-derived 3D point clouds: comparison of 4 different methods considering fruit occlusions [1]](http://www.grap.udl.cat/en/publications/index.html) (submitted, not publicly available yet).

## Preparation 

First of all, create a new project folder:
```
mkdir new_project
```

Then, clone the code inside “new_project” folder:
```
cd new_project
git clone https://github.com/GRAP-UdL-AT/apple_size_estimation_in_3D_point_clouds.git
```

### Prerequisites

* MATLAB R2020a (we have not tested it in other matlab versions)
* Computer Vision System Toolbox
* Statistics and Machine Learning Toolbox

### Data Preparation

Inside the “new_project” folder, save the dataset folder “PFuji-Size_dataset” available at [PFuji-Size dataset](http://www.grap.udl.cat/en/publications/PFuji-Size_dataset.html). (not publicly available yet. It will be made publicly available after the corresponding publication acceptance)

### Launch the code

* Execute the file `/new_project/apple_size_estimation_in_3D_point_clouds/test_apple_size_estimation_in_3D_point_clouds.m`


## Authorship

This project is contributed by [GRAP-UdL-AT](http://www.grap.udl.cat/en/index.html).

Please contact authors to report bugs @ jordi.genemola@udl.cat


## Citation

If you find this implementation or the analysis conducted in our report helpful, please consider citing:

    @article{Gené-Mola2021a,
        Author = {{Gen{\'e}-Mola, Jordi and Sanz-Cortiella, Ricardo and Rosell-Polo, Joan R and Escol{\`a}, Alexandre and Gregorio, Eduard },
        Title = {In-field apple size estimation using photogrammetry-derived 3D point clouds: comparison of 4 different methods considering fruit occlusions},
        Journal = {Computers and Electronics in Agriculture},
        Year = {2021}
        doi = {https://doi.org/10.1016/j.compag.2021.106343}
    }
    
    @article{Gené-Mola2021b,
        Author = {{Gen{\'e}-Mola, Jordi and Sanz-Cortiella, Ricardo and Rosell-Polo, Joan R and Escol{\`a}, Alexandre and Gregorio, Eduard },
        Title = {PFuji-Size dataset: A collection of images and photogrammetry-derived 3D point clouds with ground truth annotations for Fuji apple detection and size estimation in field conditions},
        Journal = {Data in Brief},
        Year = {2021}
        doi = {https://doi.org/10.1016/j.dib.2021.107629}
    }
