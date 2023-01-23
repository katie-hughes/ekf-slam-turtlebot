# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1. Pass in the vector by reference, and normalize it in place
      2. Pass in the vector by value, and return a new normalized vector
      3. Add two more struct parameters, norm_x and norm_y, that you update every time x and/or y update. Then, to get the normalized vector, you simply read from those elements.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      1. A pro of passing by reference is you save space. A con is that you lose the values and magnitute of the original vector. 
      2. A pro of passing by value is you maintain the original vector in case you want to do other calculations with it. A con is that it is less space efficient.
      3. A pro of adding a new set of normalized struct variables is that you don't have to do any calculation in the moment to get the norm, it is simply a lookup. A con is you are responsible for making sure these variables get updated accordingly every time the vector is updated. Additionally, you would not want these to be public variables, since you do not want the user modifying it. This is also probably overkill as it is not that computationally difficult to normalize on the fly.

   - Which of the methods would you implement and why?
     - I implemented option 2 above of passing the vector in by value and returning a new vector.

2. What is the difference between a class and a struct in C++?

A struct's members are public by default, while class's members are private by default.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
inv() is const because it does not modify the underlying Transform2D. Instead, it returns a new Transform2D object. The *= operator is not const as it modifies the underlying Transform2D by applying another Transform2D to it. Because of this difference, the *= operator also returns the initial Transform2D object.