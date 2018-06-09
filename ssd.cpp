#include "mview.h"
#include <stdio.h>
#include <math.h> 

//compute ssd for 2 gray scale images
float ssd_cost_gray(ImageView left, ImageView right){
    assert((left.height==right.height)&&("Two imageviews pass into ssd_cost don't have the same height"));
    assert((left.width==right.width)&&("Two imageviews pass into ssd_cost don't have the same width"));
    float cost=0.0;
    for (int j=0;j<left.height;j++){
        for (int i=0;j<left.width;i++){
            cost=cost+pow((
                left.image.Gray_pixel_values((left.left_column+i),(left.top_row+j))
                -right.image.Gray_pixel_values((right.left_column+i),(right.top_row+j))
            ),2);
        }
    }
    return cost;
    
}

//compute ssd for 2 RGB images
float ssd_cost_RGB(ImageView left, ImageView right){
    assert((left.height==right.height)&&("Two imageviews pass into ssd_cost don't have the same height"));
    assert((left.width==right.width)&&("Two imageviews pass into ssd_cost don't have the same width"));
    float cost=0.0;
    for (int j=0;j<left.height;j++){
        for (int i=0;j<left.width;i++){
            for (int k=0;k<3;k++){
                cost=cost+pow((
                    left.image.RGB_pixel_values((left.left_column+i),(left.top_row+j))(k)
                    -right.image.RGB_pixel_values((right.left_column+i),(right.top_row+j))(k)
                    ),2);
            } 
        }
    }
    return cost;
    
}

