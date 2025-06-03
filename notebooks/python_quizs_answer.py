# Q1: Define variables and print them
student_name = "John Doe"
student_age = 17
print("My name is {} and I am {} years old.".format(student_name, student_age))
print(f"My name is {student_name} and I am {student_age} years old.")

# Q2: Calculate and print average score
math_score = 85
science_score = 90
average_score = (math_score + science_score) / 2
print(f"Average Score: {average_score}")

# Q3: Work with lists
subjects = ["Math", "Science", "English"]
grades = ("A", "B+", "A-")
subjects.append("History")
print(f"Updated Subjects: {subjects}")

# Q4: Create and modify a dictionary
student_info = {"name": "Alice", "age": 20, "major": "Computer Science", "GPA": 3.5}
student_info["GPA"] = 3.8
student_info["year"] = 2
print(f"Student Info:\n{student_info}")

# Q5: Create and manipulate a set
unique_subjects = {"Math", "Science", "English", "Math"}
print(f"Unique Subjects: {unique_subjects}")
unique_subjects.add("History")
print(f"Updated Set: {unique_subjects}")


# Q6: Define and call a function to calculate grade
def calculate_grade(score):
    if score >= 90:
        return "A"
    elif score >= 80:
        return "B"
    elif score >= 70:
        return "C"
    elif score >= 60:
        return "D"
    else:
        return "F"


print(calculate_grade(85))  # Expected output: B
print(calculate_grade(70))  # Expected output: C
print(calculate_grade(95))  # Expected output: A
print(calculate_grade(55))  # Expected output: F


# Q7: Use if-elif-else statements to provide feedback based on grade
student_grade = calculate_grade(85)
if student_grade == "A":
    print("Excellent work!")
elif student_grade == "B":
    print("Good job!")
else:
    print("Keep trying!")

# Q8: Implement a while loop for a countdown
countdown = 5
while countdown > 0:
    print(f"Countdown: {countdown}")
    countdown -= 1
print("Blast off!")


# Q9: Define a class and create an object
class Student:
    def __init__(self, name, age, average_score):
        self.name = name
        self.age = age
        self.average_score = average_score

    def display_info(self):
        print(
            f"Name: {self.name}, Age: {self.age}, Average Score: {self.average_score}"
        )

    def update_score(self, new_score):
        self.average_score = new_score


student1 = Student("Alice Smith", 16, 88.5)
student1.display_info()
student1.update_score(90.0)
student1.display_info()


# Q10: Define a subclass and create an object
class HighSchoolStudent(Student):
    def __init__(self, name, age, average_score, grade_level):
        # Call the constructor of the parent class
        super().__init__(name, age, average_score)
        # Add a new instance attribute for the grade level
        self.grade_level = grade_level

    def display_info(self):
        # Call the display_info method of the parent class
        super().display_info()
        print(f"Grade Level: {self.grade_level}")


high_school_student = HighSchoolStudent("Bob Johnson", 17, 85.0, 11)
high_school_student.display_info()
