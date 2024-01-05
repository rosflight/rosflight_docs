ROSflight is intended to be a streamlined, bare-bones autopilot. We welcome any bug fixes, cleanup, or other contributions which do not add complexity or detract from the readability and simplified nature of the project. In an attempt to avoid "feature creep," we will be very discriminatory in merging pull requests whose purpose is to simply add features. Forking the repository in order to add features is totally acceptable and encouraged, just recognize us as the original authors of the autopilot (per the agreement in the BSD-3 license).

In addition, we strive to maintain a very high standard of quality in terms of code style, variable naming, and the like. By maintaining a high standard, we hope that the code will continue to be useful, understandable, and cohesive in nature. Please don't be offended if we ask you to modify the formatting of your code before approving a pull request.

Although we strive for complete in-code documentation, in practice this sometimes gets left behind for the sake of rapid development. If you, as a potential developer, find some portion of documentation unsatisfactory, we welcome questions on the appropriate GitHub issues page or [forum](https://discuss.rosflight.org/), and encourage you to submit pull requests which improve documentation. Several new developers have started with first improving the documentation to get a handle on how things work.

## Key Goals of ROSflight

Here is a summary of the key goals and philosophies behind ROSflight. As you look to make contributions to the project, keep these in mind.

- Only include the things that most people will need. The goal of this would be to do most of the work so people can get a MAV in the air quickly and easily, but not overcomplicate the code with features that only a small portion of users would need.

- Be modular and adaptable for many research-centric use cases. This will be accomplished by putting the majority of the autopilot in a well-designed ROS2 framework. That which needs to be on the microcontroller will need to be done so carefully with good coding practices. Additionally, microcontroller code that is the most likely to be expanded upon should include clear interfaces and instructions for doing so.

- Keep everything simple and well documented. The key goal here is to minimize the amount of time and effort it takes someone to go from knowing nothing about ROSflight to being able to implement their own features and making meaningful progress with their research.

## Communication

There are two channels to communicate with the developer team. For bug reports, feature requests, and anything to do with code, please open an issue on the appropriate [GitHub](https://github.com/rosflight) issue page. For questions and other discussions, please use the [forum](https://discuss.rosflight.org/).
