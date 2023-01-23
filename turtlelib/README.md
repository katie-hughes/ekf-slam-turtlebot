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
     - I implemented option 2 above of passing the vector in by value and returning a new vector. I liked that this maintained the original vector, as opposed to option 1. Additionally, it was simpler to implement than keeping track of new variables as mentioned in option 3.

2. What is the difference between a class and a struct in C++?

A struct's members are public by default, while class's members are private by default. Other than this, they can be used in the exact same way.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

Vector2D is a struct because it is useful for the user be able to read/modify the x and y components. Transform2D is a class because there are certain constraints to the rotation that you don't want the user modifying at will. Once you have created the Transform2D, you only want to modify it in very specific ways (for example, applying another Transform2D to it). This relates to core guideline C.2: Use class if the class has an invariant; use struct if the data members can vary independently. 

Additionally, Vector2D is mainly used as a way of grouping data. There is no need to have private members. Transform2D, as mentioned earlier, has private members that you don't want the user to directly modify. This relates to core guideline C.8: Use class rather than struct if any member is non-public.

<!-- F.21: To return multiple “out” values, prefer returning a struct or tuple -->

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

C.46: By default, declare single-argument constructors explicit as to avoid unintended conversions. For example, we have two constructors for Transform2D that take in one argument (one takes in a Vector2D and the other takes in a double). The explicit keyword in front of these is to ensure that they get used with the correct type.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

inv() is const because it does not modify the underlying Transform2D. Instead, it returns a new Transform2D object. The *= operator is not const as it modifies the underlying Transform2D by applying another Transform2D to it. Because of this difference, the *= operator also returns the initial Transform2D object.