Parsing to run StaticFusion 

===

## How to use

1. rgb와 rgbd를 .png파일로 저장해야 함

***rgb***: image가 BGR이 아닌 RGB로 저장해야 StaticFusion을 돌렸을 때 정상 칼라로 나옴
***depth***: mm단위로 저장된 uint16 monochrome 이미지로 저장해야 함! (확장자 마찬가지로 .png)

2. StaticFusion 코드를 보니 rgbd_assoc.txt가 꼭 0부터 시작할 필요는 없어 보임! 근데 timestamp를 double로 받아오는 것 같아서, 0 padding을 추가했다가 뺌
## rgbd depth

1. CV_32FC1로 저장하는 방법, CV_16UC1로 저장하는 방법 두가지가 존재함

* CV_32FC1: `float`으로 m로 저장함.
* CV_16UC1: `unsigned short`로 mm로 저장함. <= 저장공간을 좀 덜 쓰니 이 방식으로 저장된게 아닐까 생각됨

출력하면 각각 

* CV_16UC1 - 2
* CV_32FC1 - 5

2. nan을 처리하는 법

C++에는 `std::isnan($value$)`를 통해 pixel-wise value가 nan인지 아닌지 판별할 수 있음.

StaticFusion에 업로드해 둔 depth는 nan을 0으로 채운듯 함

## Note

**Eigensolver couldn't find a solution. Pose is not updated** 라는 경고가 뜸 (왜지??)

=> 현재 파일이 uint16이었는데, 확인해보니 uint8로 불러와지고, min, max가 이상하게 되어있음!

