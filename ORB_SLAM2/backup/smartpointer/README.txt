This two folders contains the code in which most of the pointers
are generated as "shared_ptr","weak_ptr" or "unique_ptr". 
This solved the memory leak issue to some extent.

Also, saving map and loading map are added via boost serialization.
