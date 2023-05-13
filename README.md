<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">


  <h1 align="center">Human Detection and Tracking using m500 quadrotor </h1>


</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary><h3>Table of Contents</h3></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#demo">Demo</a></li>
      </ul>
    </li>
    <li>
      <a href="#report">Report</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributors">Contributors</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This project deals with the implementation of human detection and tracking. The yaw is controlled using bounding box info. 
Depth info is used for forward and backward motion.

Summary of tasks achieved:
* Implemented ROS node for accessing and viewing Yolo v5 output.
* Controlled yaw angle based on bounding box center.
* Setup 1D LiDAR for forward and backward motion.
* Programmed PD controllers for both motions. 


<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Demo

<div align="center">


  <h4 align="center"> Follow Me mode on Drone (X3 Speed)</h4>


</div>

[https://user-images.githubusercontent.com/90359587/224387441-d45e0f85-1992-43dc-be13-360b4ef2d11c.mp4](https://github.com/KACHAPPILLY2021/Follow_me_ModalAI-m500/assets/90359587/13569a64-39f3-4366-a2b0-342871befb7d)

[![Youtube](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/VT-2gj4TgY8)

<div align="center">


  <h4 align="center"> Host Computer View (X3 Speed)</h4>


</div>

https://github.com/KACHAPPILLY2021/Follow_me_ModalAI-m500/assets/90359587/bead7b87-0798-49b9-9573-17ba9fd01831

[![Youtube](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/9CFo-q8fig8)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- Document and Reports -->
## Report

Detailed decription for this project can be found in this [Report]()
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

These are the instructions to get started on the project.
To get a local copy up and running follow these simple steps.

### Prerequisites
* OS - Linux (tested)
* Object Detection model - Yolo v5
* Hardware - Hires Camera, 1D LiDAR
* Software - PX4, ROS 1, C++
* Package required - voxl_mpa_to_ros
* Docker - ROS melodic with opencv1.2 (mavros and mavros_extras required) 
* Ground Station - Qgroundcontrol

### Installation

1. Add the following inside ```bashrc``` of your local machine.
   ```sh
   export ROS_MASTER_URI=<Drone_IP>
   ```
2. Setup ```tflite``` service for yoloV5 and select hires camera.
3. Clone  package ```human_tracking``` into ```src``` of workspace and ```catkin_make```.


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

For starting ```offboard_mode```. Open 4 terminals and do the following.
1. First terminal. ```ssh``` into drone and source ROS variables. Then enter following command to start ```voxl_mpa_to_ros```
   ```sh
   roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
   ```
2. Second terminal. Get into docker image and source the workspace and run following to start controller. 
   ```sh
   roslaunch human_tracking follower.launch
   ```
3. Third terminal. Get into docker image and source the workspace and run following to view camera output with bounding boxes. 
   ```sh
   rosrun human_tracking view
   ```
4. Fourth terminal. Source ```.bashrc``` of local machine and run ```rqt``` and select following topic for image view. 
   ```sh
   /human_bb_view
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTORS -->
## Contributors
- [Jeffin Johny](https://github.com/KACHAPPILLY2021)
- [Pradip Kathiriya](https://github.com/Pradip-Kathiriya)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Jeffin Johny K - [![MAIL](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:jeffinjk@umd.edu)
	
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://kachappilly2021.github.io/)
[![linkedin](https://img.shields.io/badge/linkedin-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](http://www.linkedin.com/in/jeffin-johny-kachappilly-0a8597136)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [MIT](https://choosealicense.com/licenses/mit/) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com
