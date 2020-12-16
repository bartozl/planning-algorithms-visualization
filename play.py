from world import world
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-H", type=int, default=20)
parser.add_argument("-W", type=int, default=20)
parser.add_argument("-P", type=int, default=800)
parser.add_argument("-M", type=int, default=1)
parser.add_argument("-sbs", action='store_true', default=False)
parser.add_argument("-gif", action='store_true', default=False)

args = parser.parse_args()
height, width, margin, pixels, sbs, gif = args.H, args.W, args.M, args.P, args.sbs, args.gif
game = world(height=height, width=width, margin=margin, pixels=pixels, step_by_step=sbs, gif=gif)
game.run()
