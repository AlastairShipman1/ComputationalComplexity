from visualisation.PygameVisualiser.PygameVisualisation import Visualisation

def main():
    v=Visualisation()
    v.run()

#TODO: put in constant curvature motion planning (possibly a green arrow?)
#TODO: find a way of getting the nearest point in an unknown region
#TODO: then do the maths bit of closest time approach given worst case agent trajectory
#TODO: then finally do the perception/control time (replanning?) and velocity limiting
if __name__ == "__main__":
    main()
