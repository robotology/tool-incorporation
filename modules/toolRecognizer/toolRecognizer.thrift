#toolRecognizer.thrift

/**
* toolRecognizer_IDLServer
*
* Interface. 
*/


struct BoundingBox
{
  1: i32 tlx;
  2: i32 tly;
  3: i32 brx;
  4: i32 bry;
}

service toolRecognizer_IDLServer
{
    /**
     * Start the module
     * @return true/false on success/failure
     */
    bool start();

    /**
     * Quit the module
     * @return true/false on success/failure
     */
    bool quit();

    /**
     * Command to train tools by their label
     * @return true/false on success/failure to train classifiers.
     */
    bool train(1: string label, 2:i32 tlx = 0, 3:i32 tly = 0, 4:i32 brx = 0 , 5:i32 bry = 0);

    /**
     * Classifies image into one of the learned tool categories.
     * @return label of recognized tool class.
     */
    string recognize(1:i32 tlx = 0, 2:i32 tly = 0, 3:i32 brx = 0 , 4:i32 bry = 0);

    /**
     * Checks whether the hand is full or empty
     * @return true/false  corresponding to full or empty hand
     */
    bool burst(1:bool burstF);
}
