from visualisation.PygameVisualiser.PygameVisualisation import Visualisation

def main():
    v=Visualisation()
    v.run()

# TODO: then finally do the perception/control time (replanning?) and velocity limiting
# TODO: make a zoom and pan function
# TODO: draw a line showing worst case movement from nearest unknown point
#
        # TODO: draw vector of worst case actor movement
        # TODO: get dynamic obstacles to show shadows
if __name__ == "__main__":
    main()
