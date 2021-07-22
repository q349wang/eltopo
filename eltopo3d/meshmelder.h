// ---------------------------------------------------------
//
//  meshmerger.h
//  Tyson Brochu 2011
//  
//  Search for mesh edges which are near to each other, zipper their neighbouring triangles together.
//
// ---------------------------------------------------------

#ifndef EL_TOPO_MESHMELDER_H
#define EL_TOPO_MESHMELDER_H

// ---------------------------------------------------------
//  Nested includes
// ---------------------------------------------------------

#include <meshoperator.h>
#include <vector>
#include <vec.h>

// ---------------------------------------------------------
//  Forwards and typedefs
// ---------------------------------------------------------

class SurfTrack;

// ---------------------------------------------------------
//  Class definitions
// ---------------------------------------------------------

// ---------------------------------------------------------
///
///
///
// ---------------------------------------------------------

class MeshMelder : public MeshOperator
{
    
public:
    
    MeshMelder( SurfTrack& surf ) :
      MeshOperator( surf )
    {}
    
    /// 
    ///
    void process_mesh();

private:

};


#endif
