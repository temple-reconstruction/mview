#include "mview.h"
#include <stdio.h>
#include <math.h> 

float ssd_cost(ImageView left, ImageView right){
    assert((left.height==right.height)&&("Two imageviews pass into ssd_cost don't have the same height"));
    assert((left.width==right.width)&&("Two imageviews pass into ssd_cost don't have the same width"));
    float cost=0.0;
    for (int j=0;j<left.height;j++){
        for (int i=0;j<left.width;i++){
            for (int k=0;k<3;k++){
                cost=cost+pow((
                    left.image.pixel_values(((left.top_row+j)*left.width+left.left_column+i),k)
                    -right.image.pixel_values(((right.top_row+j)*right.width+right.left_column+i),k)
                    ),2);
            } 
        }
    }
    return cost;
    
}