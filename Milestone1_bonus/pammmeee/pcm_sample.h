/*********************************************************************
 * Milestone1 
 * Authors: Chousos Christos AM:2012030117
 *	        Giariskanis Fotios AM:2011030087
 *
 *********************************************************************/

const long pcm_length=6800;

const unsigned char pcm_samples[] PROGMEM ={
   200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
	12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
	    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
	12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
	12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
	12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
	12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	20, 39, 59, 100, 110, 125, 128, 126, // 0-7
	18, 29, 39, 44, 55, 105, 128, 126, // 0-7
	  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12, 
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
      1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	 1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	  1, 2, 3, 4, 5, 10, 128, 126, // 0-7
	   1, 2, 3, 4, 5, 10, 128, 126, // 0-7
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
    12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7
   12, 10, 7, 4, 30, 50, 70, 110, // 0-7  
  12, 10, 7, 4, 30, 50, 70, 110, // 0-7
  110, 70, 50, 30, 4, 7, 10, 12,
   126, 126, 128, 126, 128, 126, 126, 126, // 8-15
126, 126, 128, 126, 126, 128, 126, 126, // 16-23
126, 128, 126, 126, 128, 126, 126, 126, // 24-31
128, 126, 126, 126, 127, 126, 126, 127, // 32-39
127, 126, 125, 128, 126, 125, 126, 126, // 40-47
126, 126, 125, 128, 125, 126, 129, 126, // 48-55
126, 126, 125, 126, 127, 121, 125, 125, // 56-63
126, 126, 126, 125, 126, 126, 126, 128, // 64-71
126, 125, 128, 129, 128, 126, 126, 129, // 72-79
128, 127, 128, 127, 125, 125, 127, 131, // 80-87
126, 128, 130, 128, 128, 132, 131, 127, // 88-95
128, 129, 131, 127, 132, 129, 125, 131, // 96-103
131, 129, 132, 131, 126, 125, 128, 129, // 104-111
128, 129, 128, 126, 130, 125, 132, 128, // 112-119
129, 129, 129, 124, 129, 127, 123, 129, // 120-127
122, 129, 120, 135, 126, 127, 126, 132, // 128-135
122, 130, 124, 126, 133, 122, 132, 126, // 136-143
121, 129, 129, 122, 133, 125, 129, 133, // 144-151
123, 129, 131, 122, 128, 127, 131, 136, // 152-159
117, 130, 130, 118, 130, 130, 126, 128, // 160-167
127, 128, 131, 124, 122, 126, 129, 133, // 168-175
119, 129, 132, 122, 125, 129, 124, 126, // 176-183
124, 132, 132, 116, 128, 131, 122, 119, // 184-191
136, 123, 127, 123, 129, 125, 121, 128, // 192-199
134, 121, 129, 124, 131, 118, 130, 135, // 200-207
122, 126, 130, 122, 124, 137, 120, 137, // 208-215
125, 137, 120, 128, 129, 127, 127, 130, // 216-223
120, 132, 130, 125, 133, 119, 132, 128, // 224-231
111, 114, 112, 117, 124, 123, 124, 128, // 4180952-4180959
124, 127, 126, 130, 130, 127, 136, 130, // 4180960-4180967
136, 136, 132, 137, 139, 136, 144, 143, // 4180968-4180975
147, 137, 142, 139, 144, 142, 137, 148, // 4180976-4180983
152, 149, 144, 143, 145, 141, 133, 141, // 4180984-4180991
144, 145, 142, 141, 137, 135, 134, 142, // 4180992-4180999
143, 138, 134, 132, 131, 127, 131, 122, // 4181000-4181007
122, 127, 122, 124, 118, 122, 120, 115, // 4181008-4181015
115, 114, 111, 118, 118, 111, 111, 104, // 4181016-4181023
112, 113, 114, 121, 119, 112, 109, 100, // 4181024-4181031
 99, 106, 105, 103, 102, 105, 101,  98, // 4181032-4181039
104,  98,  99,  96,  96,  94,  88,  95, // 4181040-4181047
100,  94,  90,  89,  85,  85,  94,  97, // 4181048-4181055
 90,  86,  83,  84,  85,  90,  94,  90, // 4181056-4181063
 86,  88,  86,  85,  83,  78,  86,  90, // 4181064-4181071
 84,  86,  85,  84,  88,  88,  86,  90, // 4181072-4181079
 92,  87,  90,  92,  98,  92,  93, 101, // 4181080-4181087
100, 103, 106, 108, 108, 109, 112, 110, // 4181088-4181095
116, 116, 115, 127, 127, 132, 129, 128, // 4181096-4181103
126, 132, 132, 132, 139, 143, 144, 137, // 4181104-4181111
134, 134, 140, 139, 139, 140, 145, 140, // 4181112-4181119
138, 140, 143, 145, 141, 138, 142, 143, // 4181120-4181127
137, 134, 140, 132, 137, 137, 136, 135, // 4181128-4181135
132, 127, 127, 126, 127, 124, 118, 122, // 4181136-4181143
123, 121, 119, 115, 110, 108, 109, 112, // 4181144-4181151
115, 110, 106, 107, 100, 102, 102, 100, // 4181152-4181159
104, 106, 107, 106, 106,  99,  96, 101, // 4181160-4181167
104, 104, 102,  99, 104, 102, 101, 107, // 4181168-4181175
105, 105, 103, 111, 105, 104, 107, 110, // 4181176-4181183
113, 117, 117, 114, 114, 114, 109, 116, // 4181184-4181191
108, 113, 120, 116, 121, 120, 118, 118, // 4181192-4181199
118, 120, 121, 119, 120, 117, 129, 124, // 4181200-4181207
123, 122, 119, 124, 120, 120, 125, 122, // 4181208-4181215
119, 130, 128, 124, 121, 119, 123, 128, // 4181216-4181223
130, 127, 124, 124, 123, 125, 128, 131, // 4181224-4181231
133, 129, 131, 135, 135, 133, 129, 135, // 4181232-4181239
136, 131, 139, 144, 139, 139, 142, 147, // 4181240-4181247
150, 149, 145, 153, 145, 145, 144, 151, // 4181248-4181255
152, 154, 160, 157, 153, 153, 151, 148, // 4181256-4181263
150, 156, 153, 154, 153, 149, 152, 149, // 4181264-4181271
147, 149, 144, 138, 140, 147, 145, 139, // 4181272-4181279
135, 136, 133, 131, 130, 131, 131, 130, // 4181280-4181287
122, 119, 116, 112, 116, 119, 114, 112, // 4181288-4181295
113, 107, 102,  99, 101, 101,  94,  98, // 4181296-4181303
 96,  93,  93,  94,  94,  90,  92,  91, // 4181304-4181311
 93,  86,  89,  86,  83,  86,  89,  89, // 4181312-4181319
 89,  91,  94,  97,  94,  95, 105, 101, // 4181320-4181327
 96,  97, 101, 106, 101, 105, 106, 103, // 4181328-4181335
103, 106, 111, 112, 116, 118, 113, 116, // 4181336-4181343
120, 118, 117, 118, 124, 122, 120, 122, // 4181344-4181351
126, 124, 123, 128, 132, 127, 127, 126, // 4181352-4181359
133, 131, 124, 127, 128, 122, 125, 132, // 4181360-4181367
131, 129, 129, 126, 125, 124, 124, 125, // 4181368-4181375
128, 124, 123, 121, 119, 122, 125, 127, // 4181376-4181383
129, 126, 124, 116, 114, 119, 126, 124, // 4181384-4181391
125, 129, 126, 122, 124, 123, 121, 121, // 4181392-4181399
123, 124, 126, 120, 120, 126, 119, 118, // 4181400-4181407
127, 126, 121, 122, 123, 122, 119, 118, // 4181408-4181415
122, 117, 118, 121, 118, 118, 117, 117, // 4181416-4181423
116, 113, 114, 116, 112, 110, 114, 115, // 4181424-4181431
105, 102, 104, 104, 107, 104, 109, 109, // 4181432-4181439
103,  97,  98, 100, 100,  99, 100,  97, // 4181440-4181447
 95,  95,  98,  91,  96,  97,  94,  88, // 4181448-4181455
 88,  97,  95,  91,  95,  93,  93,  89, // 4181456-4181463
 95,  98,  98, 102, 104, 101,  98, 100, // 4181464-4181471
104, 106, 106, 111, 111, 111, 112, 107, // 4181472-4181479
114, 115, 119, 120, 121, 122, 122, 121, // 4181480-4181487
122, 127, 134, 131, 129, 127, 130, 133, // 4181488-4181495
133, 133, 136, 133, 138, 136, 137, 142, // 4181496-4181503
140, 139, 142, 142, 138, 134, 138, 137, // 4181504-4181511
138, 142, 142, 138, 135, 137, 140, 139, // 4181512-4181519
140, 137, 135, 133, 136, 138, 136, 136, // 4181520-4181527
141, 143, 135, 136, 138, 137, 140, 145, // 4181528-4181535
147, 143, 139, 142, 140, 145, 146, 152, // 4181536-4181543
152, 145, 146, 149, 146, 147, 150, 146, // 4181544-4181551
147, 150, 153, 149, 150, 155, 146, 149, // 4181552-4181559
151, 151, 149, 150, 152, 151, 147, 147, // 4181560-4181567
146, 146, 145, 149, 145, 144, 144, 139, // 4181568-4181575
138, 137, 138, 134, 130, 133, 131, 136, // 4181576-4181583
139, 130, 132, 132, 129, 128, 128, 131, // 4181584-4181591
133, 127, 126, 130, 132, 128, 128, 131, // 4181592-4181599
128, 129, 130, 132, 136, 134, 137, 132, // 4181600-4181607
133, 137, 139, 141, 143, 144, 146, 145, // 4181608-4181615
145, 144, 152, 153, 156, 164, 160, 153, // 4181616-4181623
156, 159, 159, 162, 166, 170, 171, 174, // 4181624-4181631
178, 175, 171, 176, 179, 180, 180, 182, // 4181632-4181639
184, 182, 178, 180, 182, 181, 183, 182, // 4181640-4181647
187, 188, 179, 177, 178, 174, 174, 177, // 4181648-4181655
181, 181, 176, 175, 175, 174, 167, 167, // 4181656-4181663
170, 166, 165, 166, 166, 160, 158, 158, // 4181664-4181671
156, 158, 156, 155, 158, 148, 151, 149, // 4181672-4181679
149, 153, 151, 150, 147, 144, 143, 145, // 4181680-4181687
145, 149, 148, 150, 146, 144, 141, 145, // 4181688-4181695
148, 147, 149, 147, 146, 147, 143, 145, // 4181696-4181703
147, 150, 150, 149, 153, 156, 151, 149, // 4181704-4181711
150, 148, 147, 148, 147, 148, 152, 153, // 4181712-4181719
149, 147, 143, 137, 139, 140, 138, 138, // 4181720-4181727
137, 137, 134, 130, 132, 131, 127, 133, // 4181728-4181735
137, 129, 127, 131, 126, 124, 124, 123, // 4181736-4181743
};
