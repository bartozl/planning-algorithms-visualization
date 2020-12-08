from world import world
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-H", type=int, default=20)
parser.add_argument("-W", type=int, default=20)
parser.add_argument("-P", type=int, default=800)
parser.add_argument("-M", type=int, default=1)

args = parser.parse_args()
height, width, margin, pixels = args.H, args.W, args.M, args.P

game = world(height=height, width=width, margin=margin, pixels=pixels)
game.run()
