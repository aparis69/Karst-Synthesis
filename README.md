## Synthesizing Geologically Coherent Cave Networks

<img src="https://aparis69.github.io/public_html/imgs/karsts_representative.jpg"
     alt="Cave Networks - representative image"
     style="float: left; margin: 5px;" />
	 
Source code for our paper "Synthesizing Geologically Coherent Cave Networks" published in Computer Graphics Forum in 2021 and presented at Pacific Graphics 2021. This is aimed at researchers, students or profesionnals who may want to reproduce **some** of the results described in the paper.

[Project page](https://aparis69.github.io/public_html/projects/paris2021_Karsts.html)

[Paper](https://hal.archives-ouvertes.fr/hal-03331697/file/2021-caves-author.pdf)

### Important notes
* This code is **not** the one which produced the scenes seen in the paper. Everything has been *recoded* on my side to make sure it is free to use. Hence, the results as well as the timings may differ from the ones in the paper.
* This is **research** code provided without any warranty. However, if you have any problem you can still send me an email or create an issue.

### Testing
There is no dependency. Running the program will output 3 karstic skeletons stored in ASCII files. Each scene consists of 2 files: a node file and a link file. The format is easy to undertand.
Tests have been made on:
* Visual Studio 2019: double click on the solution in ./VS2019/ and Ctrl + F5 to run
* Ubuntu 16.04: cd ./G++/ && make && ./Out/KarstSynthesis

In you can't compile or run the code, results produced with this code are available in the Results/ folder.

### Citation
You can use this code in any way you want, however please credit the original article:
```
@article {Paris2021,
    author = {Paris, Axel and Guérin, Eric and Peytavie,
              Adrien and Collon, Pauline and Galin, Eric},
    title = {Synthesizing Geologically Coherent Cave Networks},
    journal = {Computer Graphics Forum},
    volume = {40},
    number = {7},
    year = {2021},
  }
```	

### Missing
There are still some things missing from the paper implementation. They might be added in the future if someone is interested. What is not in the code:
* Midpoint subdivision and classification of the network
* Implict surface generation
