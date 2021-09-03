# This program draws the Archimedean Spiral, starting from a central point and
# spiralling outwards.

from turtle import Turtle, Screen
from math import pi, sin, cos
from random import randint, random

RADIUS = 180

screen = Screen()

WIDTH = screen.window_width()
HEIGHT = screen.window_height()

turtle = Turtle(visible=False)
turtle.speed('fastest')

turtle.up()

# x and y are random start point co-ordinates for the centre
x = randint(RADIUS - WIDTH//2, WIDTH//2 - RADIUS)
y = randint(RADIUS - HEIGHT//2, HEIGHT//2 - RADIUS)
turtle.goto(x, y)

turtle.down()


for i in range(20000):

	# t stands for Theta in this example, which directly defines how 'smooth'
	# the spiral generated will be. Outside of the iterator and the pi value,
	# there is a hyperparameter constant that is used to define how small 
	# we want theta.
	#
	# With smaller values (like 5), this will generate a result that is
	# polygonal moreso than a conventional circular motion. With larger values
	# (like 2000), the shape will take far longer to generate but it will be 
	# much more akin to a perfect circle.
	#
	# A good common ground for this divisor is something like 30; pi of course
	# will stay the same and the iterator shall slowly increase this value over
	# time, making the spiral larger.

	t = i / 30 * pi

	# We follow the rule of r = a + b * theta, and then we apply the cos
	# function to dx and sin function to dy respectively.
	#
	# initial value of a: 1, initial value of b: 5.
	#
	# a = how far away a straight line is drawn from the chosen centre until
	# the spiralling begins.
	#
	# b = the distance between the current spiral radial position and the
	# previous point that was made at that exact radial position before. In
	# other words, this defines exactly how 'wide' the spiral motion is.
	#
	# Although initially integers were used for both a and b, they can be ANY
	# REAL NUMBER. Usually you will want a to be relatively a lot smaller than
	# b.
	#
	# We multiply by their cos and sin functions as this ranges from between 
	# 1 and -1 per every pi interval, where sin starts at 0 (increasing) at 
	# origin but cos starts at 1. This means that together, they will move the
	# dimensions around in a circular motion when r is multiplied by this.

	dx = (0.1 + 3.75 * t) * cos(t)
	dy = (0.1 + 3.75 * t) * sin(t)

	turtle.goto(x + dx, y + dy)

turtle.up()
