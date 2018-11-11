import PRM
import pianoNodes

def main():
    twoDnodes, twoDadjacency, twoDdistances=PRM.PRM2D(100)
    #print(twoDnodes)
    #print("derp")
    #print(twoDadjacency)
    #print("derp")
    #print(twoDdistances)
    PRM.PRM2Dshow(twoDnodes, twoDadjacency, twoDdistances)

if __name__ == "__main__":
    main()
    

