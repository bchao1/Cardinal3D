
#include <queue>
#include <set>
#include <unordered_map>
#include <iostream>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    if(v == vertices.end()) return std::nullopt; // Don't erase invalid vertices
    if(v->on_boundary()) return std::nullopt; // Don't erase boundary vertices

    std::vector<EdgeRef> adjacent_edges;
    HalfedgeRef h = v->halfedge();
    do {
        std::cout << h->edge()->id() << "\n";
        adjacent_edges.push_back(h->edge());
        h = h->twin()->next();
    } while(h != v->halfedge());

    std::optional<FaceRef> f = std::nullopt;
    for(auto e : adjacent_edges) {
        std::cout << e->id() << "\n";
        f = erase_edge(e);
        if(f == std::nullopt) {
            std::cout << "Erasing vertex failed.\n";
            return std::nullopt;
        }
    }
    return f;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    if(e->on_boundary()) return std::nullopt; // Don't erase boundary edges
    if(e == edges.end()) return std::nullopt; // Don't erase invalid edges
    
    std::cout << "erasing edge: " << e->id() << "\n";

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->twin();
    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();

    if(f0 == f1) {
        if(h0->next()->next() == h0 && h0->next() == h1) {
            erase(h0);
            erase(h1);
            erase(v0);
            erase(v1);
            erase(e);
            erase(f0);
            return std::nullopt;
        } 
        HalfedgeRef h2 = (h0->next() == h1) ? h0 : h1; // check halfedge orientation
        HalfedgeRef h3 = h2->twin();
        HalfedgeRef h4 = h2->prev();
        HalfedgeRef h5 = h3->next();
        v0 = h2->vertex();
        v1 = h3->vertex();
        f0 = h2->face();

        v0->halfedge() = h5;
        f0->halfedge() = h4;

        h4->next() = h5;

        erase(v1);
        erase(h2);
        erase(h3);
        erase(e);
        return f0;
    }

    HalfedgeRef h2 = h0->prev();
    HalfedgeRef h3 = h0->next();
    HalfedgeRef h4 = h1->prev();
    HalfedgeRef h5 = h1->next();

    v0->halfedge() = h5;
    v1->halfedge() = h3;
    f0->halfedge() = h2;

    for(HalfedgeRef h = h5; h != h1; h = h->next()) {
        h->face() = f0;
    }

    h2->next() = h5;
    h4->next() = h3;

    erase(h0);
    erase(h1);
    erase(e);
    erase(f1);
    
    return f0;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // some bugs

    (void)e;
    if(e->on_boundary()) {
        std::cout << "boundary edge, not collapsing\n";
        return std::nullopt; // Don't collapse boundary edges
    }

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->twin();

    HalfedgeRef h2 = h0->next();
    HalfedgeRef h3 = h0->prev();

    HalfedgeRef h4 = h1->next();
    HalfedgeRef h5 = h1->prev();

    HalfedgeRef h6 = h2->twin();
    HalfedgeRef h7 = h6->next();
    HalfedgeRef h8 = h6->prev();

    HalfedgeRef h9 = h4->twin();
    HalfedgeRef h10 = h9->next();
    HalfedgeRef h11 = h9->prev();

    unsigned int f0_deg = h0->face()->degree();
    unsigned int f1_deg = h1->face()->degree();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();

    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();
    FaceRef f2 = h6->face();
    FaceRef f3 = h9->face();

    std::vector<HalfedgeRef> adj_halfedges; // halfedges adjacent to v0 and v1

    HalfedgeRef ptr = v0->halfedge();
    do {
        adj_halfedges.push_back(ptr);
        ptr = ptr->twin()->next();
    } while(ptr != v0->halfedge());
    ptr = v1->halfedge();
    do {
        adj_halfedges.push_back(ptr);
        ptr = ptr->twin()->next();
    } while(ptr != v1->halfedge());

    // New vertex
    VertexRef newVertex = new_vertex();
    newVertex->pos = (v0->pos + v1->pos) / 2.0f;

    // Update adjacent halfedge connections
    for(auto h : adj_halfedges) {
        std::cout << h->id() << "\n";
        h->vertex() = newVertex;
    }
    if(f0_deg == 3) {
        h8->next() = h3;
        h3->next() = h7;
        h3->face() = f2;
        h3->vertex()->halfedge() = h3;
        f2->halfedge() = h3;
        erase(h2);
        erase(h6);
        erase(h2->edge());
        erase(f0);
    }
    else {
        h3->next() = h2;
    }
    if(f1_deg == 3) {
        h11->next() = h5;
        h5->next() = h10;
        h5->face() = f3;
        h5->vertex()->halfedge() = h5;
        f3->halfedge() = h5;
        erase(h9);
        erase(h4);
        erase(h9->edge());
        erase(f1);
    }
    else {
        h5->next() = h4;
    }

    newVertex->halfedge() = h7;

    erase(v0);
    erase(v1);
    erase(h0);
    erase(h1);
    erase(e);
    
    return newVertex;
    //return std::nullopt;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;

    unsigned int N = f->degree();

    // properties of the face
    auto halfedges = f->halfedges();
    auto edges = f->edges();
    auto vertices = f->vertices();

    std::vector<HalfedgeRef>halfedges_out;
    std::vector<HalfedgeRef>halfedges_twin;

    for(unsigned int i = 0; i < N; i++) {
        auto vertex_halfedges_out = vertices[i]->adjacent_halfedges();
        for(auto vh : vertex_halfedges_out) {
            if(vh->face() != f) {
                halfedges_out.push_back(vh);
            }
        }
        halfedges_twin.push_back(halfedges[i]->twin());
    }
    for(unsigned int i = 0; i < N; i++) {
        auto h_twin = halfedges_twin[i];
        h_twin->prev()->next() = h_twin->next();
        h_twin->face()->halfedge() = h_twin->next();
    }

    auto added_vertex = new_vertex();
    added_vertex->pos = f->center();
    added_vertex->halfedge() = halfedges_out[0];

    for(auto h : halfedges_out) {
        h->vertex() = added_vertex;
    }

    // cleanup
    for(unsigned int i = 0; i < N; i++) {
        auto h_twin = halfedges_twin[i];
        if(h_twin->next()->next() == h_twin->prev()) {
            erase(h_twin->face());
            erase_edge(h_twin->prev()->edge());
        }

        erase(halfedges[i]);
        erase(halfedges_twin[i]);
        erase(edges[i]);
    }
    
    erase(f);

    return added_vertex;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    if(e->on_boundary()) return std::nullopt; // Don't flip boundary edges

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h0->prev();
    std::cout << h0->id() << " " << h1->id() << " " << h2->id() << "\n";

    HalfedgeRef h3 = h0->twin();
    HalfedgeRef h4 = h3->next();
    HalfedgeRef h5 = h3->prev();
    std::cout << h3->id() << " " << h4->id() << " " << h5->id() << "\n";

    //HalfedgeRef h6 = h1->twin();
    //HalfedgeRef h7 = h2->twin();
    //HalfedgeRef h8 = h4->twin();
    //HalfedgeRef h9 = h5->twin();

    HalfedgeRef h10 = h1->next();
    HalfedgeRef h11 = h4->next();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h3->vertex();
    VertexRef v2 = h11->vertex();
    VertexRef v3 = h10->vertex();
    
    FaceRef f0 = h0->face();
    FaceRef f1 = h3->face();

    // assign new halfedges
    v0->halfedge() = h4;
    v1->halfedge() = h1;
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    // assign new next
    h0->next() = h10;
    h1->next() = h3;
    h3->next() = h11;
    h4->next() = h0;
    h5->next() = h1;
    h2->next() = h4;

    // assign new faces 
    h1->face() = f1;
    h4->face() = f0;

    // assign new vertices
    h0->vertex() = v2;
    h3->vertex() = v3;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    (void)e;
    
    if(e->on_boundary()) {
        HalfedgeRef h0 = (e->halfedge()->is_boundary()) ? e->halfedge() : e->halfedge()->twin();
        FaceRef f0 = h0->face();
        HalfedgeRef h1 = h0->prev();
        VertexRef v0 = h1->vertex();
        VertexRef v1 = insert_vertex(e, e->center());
        connect_vertex(v0->halfedge(), v1->get_halfedge_in_face(f0));
        return v1;
    }

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->twin();
    HalfedgeRef h2 = h0->next()->next();
    HalfedgeRef h3 = h1->next()->next();

    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();

    if(f0->degree() != 3 || f1->degree() != 3) return std::nullopt; // Don't split non-triangles

    VertexRef v2 = insert_vertex(e, e->center());
    connect_vertex(h2, v2->get_halfedge_in_face(f0));
    connect_vertex(v2->get_halfedge_in_face(f1), h3);
    return v2;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    if(v->on_boundary()) return std::nullopt; // Don't bevel boundary vertices
    std::vector<HalfedgeRef> adj_halfedges;
    HalfedgeRef h = v->halfedge();
    do {
        adj_halfedges.push_back(h);
        h = h->twin()->next();
    } while(h != v->halfedge());

    reverse(adj_halfedges.begin(), adj_halfedges.end());

    std::vector<VertexRef> new_vertices;
    for(unsigned int i = 0; i < adj_halfedges.size(); i++) {
        EdgeRef e = adj_halfedges[i]->edge();
        std::cout << "bevel edge: " << e->id() << std::endl;
        new_vertices.push_back(insert_vertex(e, e->center()));
    }
    for(unsigned int i = 0; i < adj_halfedges.size(); i++) {
        VertexRef v0 = new_vertices[i];
        VertexRef v1 = new_vertices[(i + 1) % adj_halfedges.size()];
        std::cout << "bevel vertex: " << v0->id() << " " << v1->id() << std::endl;
        FaceRef f0 = adj_halfedges[i]->face();
        std::cout << "bevel face: " << f0->id() << std::endl;
        HalfedgeRef h0 = v0->get_halfedge_in_face(f0);
        HalfedgeRef h1 = v1->get_halfedge_in_face(f0);
        connect_vertex(h0, h1);
        std::cout << "bevel vertex halfedges: " << h0->id() << " " << h1->id() << std::endl;
    }

    return erase_vertex(v);
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)f;
    if(f->is_boundary()) return std::nullopt; // Don't bevel boundary faces

    std::vector<HalfedgeRef> old_halfedges = f->halfedges();
    std::vector<VertexRef> old_vertices = f->vertices();
    std::vector<EdgeRef> old_edges = f->edges();

    int N = f->degree(); // number of vertices / edges around face

    std::vector<VertexRef> new_vertices;

    std::vector<EdgeRef> new_face_edges;
    std::vector<EdgeRef> connect_edges;

    std::vector<HalfedgeRef> new_face_halfedges;
    std::vector<HalfedgeRef> new_face_halfedges_twin;
    std::vector<HalfedgeRef> connect_halfedges_up;
    std::vector<HalfedgeRef> connect_halfedges_down;

    // allocate new data
    FaceRef new_bevel_face = new_face(); // bevelled face
    std::vector<FaceRef> new_faces_around; // new faces around bevelled face

    for(int i = 0; i < N; i++) {
        new_vertices.push_back(new_vertex());
        new_face_edges.push_back(new_edge());
        connect_edges.push_back(new_edge());
        new_face_halfedges.push_back(new_halfedge());
        new_face_halfedges_twin.push_back(new_halfedge());
        connect_halfedges_up.push_back(new_halfedge());
        connect_halfedges_down.push_back(new_halfedge());
        new_faces_around.push_back(new_face());
    }
    // go around face
    for(int i = 0; i < N; i++){
        auto next_id = (i + 1) % N;
        auto prev_id = (i + N - 1) % N;

        // original halfedge
        auto h = old_halfedges[i];
        h->face() = new_faces_around[i];
        h->edge() = old_edges[i];
        h->next() = connect_halfedges_up[next_id];
        h->vertex() = old_vertices[i];

        // new face halfedge
        auto h_new = new_face_halfedges[i];
        h_new->face() = new_bevel_face;
        h_new->edge() = new_face_edges[i];
        h_new->next() = new_face_halfedges[next_id];
        h_new->vertex() = new_vertices[i];
        h_new->twin() = new_face_halfedges_twin[i];

        // new face halfedge twin
        auto h_new_twin = new_face_halfedges_twin[i];
        h_new_twin->face() = new_faces_around[i];
        h_new_twin->edge() = new_face_edges[i];
        h_new_twin->next() = connect_halfedges_down[i];
        h_new_twin->vertex() = new_vertices[next_id];
        h_new_twin->twin() = new_face_halfedges[i];

        // new connect halfedge up
        auto h_connect_up = connect_halfedges_up[i];
        h_connect_up->face() = new_faces_around[prev_id];
        h_connect_up->edge() = connect_edges[i];
        h_connect_up->next() = new_face_halfedges_twin[prev_id];
        h_connect_up->vertex() = old_vertices[i];
        h_connect_up->twin() = connect_halfedges_down[i];

        // new connect halfedge down
        auto h_connect_down = connect_halfedges_down[i];
        h_connect_down->face() = new_faces_around[i];
        h_connect_down->edge() = connect_edges[i];
        h_connect_down->next() = old_halfedges[i];
        h_connect_down->vertex() = new_vertices[i];
        h_connect_down->twin() = connect_halfedges_up[i];

        // original twin halfedge: do nothing
        // original edge: do nothing
        // original vertex: do nothing

        // new vertices
        new_vertices[i]->halfedge() = new_face_halfedges[i];
        new_vertices[i]->pos = old_vertices[i]->pos;

        // new face edge
        new_face_edges[i]->halfedge() = new_face_halfedges[i];
        // new connect edge
        connect_edges[i]->halfedge() = connect_halfedges_up[i];

        // new faces around bevelled face
        new_faces_around[i]->halfedge() = old_halfedges[i];
        // new face
        new_bevel_face->halfedge() = new_face_halfedges[0];
    }

    erase(f); // delete old face

    return new_bevel_face;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());
    std::cout << "Bevel vertex added face: " << face->id() << std::endl;
    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    for(unsigned int i = 0; i < new_halfedges.size(); i++) {
        VertexRef v0 = new_halfedges[i]->vertex();
        VertexRef v1 = new_halfedges[i]->twin()->vertex();

        Vec3 dir = v1->pos - v0->pos;
        Vec3 shift = dir * tangent_offset;

        v0->pos += shift;
    }
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;

    // compute face normal
    auto normal = face->normal();
    auto center = face->center();

    for(unsigned int i = 0; i < new_halfedges.size(); i++) {
        auto pos = new_halfedges[i]->vertex()->pos;
        auto v = center - pos;
        v.normalize();
        auto tangent_offset_scaled = v * tangent_offset * 0.1;
        auto normal_offset_scaled = normal * normal_offset * 0.1;
        auto offset = tangent_offset_scaled + normal_offset_scaled;
        new_halfedges[i]->vertex()->pos += offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces.begin(); f != faces.end(); f++) {
        triangulate_face(f);
    }
}

void Halfedge_Mesh::triangulate_face(FaceRef f) {
    if(f->degree() == 3) return; // skip triangles
    if(f->is_boundary()) return; // skip boundary faces

    // get all halfedges
    std::vector<HalfedgeRef> halfedges;
    auto h = f->halfedge();
    do {
        halfedges.push_back(h);
        h = h->next();
    } while(h != f->halfedge());

    bool leftAdd = true;
    for(size_t left = 1, right = halfedges.size() - 1; right - left > 1; left += leftAdd, right -= !leftAdd) {
        connect_vertex(halfedges[right], halfedges[left]);
        leftAdd = !leftAdd;
    }
}


/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(auto e = edges.begin(); e != edges.end(); e++) {
        e->new_pos = e->center();
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(auto f = faces.begin(); f != faces.end(); f++) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for(auto f = faces.begin(); f != faces.end(); f++) {
        f->new_pos = f->center();
    }
    // Edges
    for(auto e = edges.begin(); e != edges.end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        FaceRef f0 = h0->face();
        FaceRef f1 = h1->face();
        VertexRef v0 = h0->vertex();
        VertexRef v1 = h1->vertex();
        e->new_pos = (f0->new_pos + f1->new_pos + v0->pos + v1->pos) / 4.0f;
    }
    // Vertices
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        unsigned int n = v->degree(); // valence
        HalfedgeRef h = v->halfedge();

        Vec3 face_sum = Vec3(0, 0, 0);
        Vec3 edge_sum = Vec3(0, 0, 0);
        do {
            // do stuff
            face_sum += h->face()->new_pos; // face pos
            edge_sum += h->edge()->center(); // midpoint
            h = h->twin()->next();
        } while(h != v->halfedge());
        face_sum /= float(n);
        edge_sum /= float(n);
        v->new_pos = (face_sum + 2.0f * edge_sum + (n - 3.0f) * v->pos) / float(n);
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.

        Halfedge_Mesh::VertexRef v0 = e->halfedge()->vertex();
        Halfedge_Mesh::VertexRef v1 = e->halfedge()->twin()->vertex();
        Mat4 Q = vertex_quadrics[v0] + vertex_quadrics[v1];
        Mat4 A = Mat4::Zero;
        Vec3 b = Vec3(0, 0, 0);
        for(unsigned int i = 0; i < 2; i++){
            for(unsigned int j = 0; j < 2; j++){
                A[i][j] = Q[i][j];
            }
        }
        for(unsigned int i = 0; i < 2; i++){
            b[i] = -Q[3][i];
        }

        if(abs(A.det()) < 1e-6) {
            optimal = edge->center();
        } else {
            optimal = A.inverse() * b;
        }
        Vec4 u = Vec4(optimal, 1);
        cost = dot(u, Q * u);
    }

    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    // Edge cases:
    // 1. very small mesh (4 faces)
    // 2. non-triangle meshes (bunny)
    // 3. edge wrapping vertex
    // 4. duplicate edges

    for(auto f = faces.begin(); f != faces.end(); f++) {
        if(f->degree() != 3) {
            std::cout << "Detected non-triangle polygons\n";
            return false;
        }
    }

    for(auto f = faces.begin(); f != faces.end(); f++) {
        std::cout << "building face quadrics\n";
        double d = -dot(f->normal(), f->halfedge()->vertex()->pos);
        Vec4 v = Vec4(f->normal(), d);
        face_quadrics[f] = outer(v, v);
    }
    
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        std::cout << "building vertex quadrics\n";
        Mat4 q = Mat4::Zero;
        auto adjacent_faces = v->adjacent_faces();
        for(auto f : adjacent_faces) {
            q += face_quadrics[f];
        }
        vertex_quadrics[v] = q;
    }

    for(auto e = edges.begin(); e != edges.end(); e++) {
        std::cout << "building edge records\n";
        Edge_Record er = Edge_Record(vertex_quadrics, e);
        edge_queue.insert(er);
        edge_records[e] = er;
    }

    unsigned int num_target_faces = faces.size() / 4;
    if(num_target_faces < 4) return false; // dirty fix for edge case 1
    while(faces.size() > num_target_faces) {
        Edge_Record best_er = edge_queue.top();
        edge_queue.pop();

        auto adjacent_edges = best_er.edge->adjacent_edges();
        for(auto e : adjacent_edges) {
            edge_queue.remove(edge_records[e]);
        }

        std::optional<VertexRef> collapse_result = collapse_edge_erase(best_er.edge);
        if(collapse_result == std::nullopt) {
            std::cout << "Mesh simplification: collapse failed.\n";
            return false;
        }
        Halfedge_Mesh::VertexRef new_vertex = std::move(*collapse_result);
        new_vertex->pos = best_er.optimal;

        Mat4 q = Mat4::Zero;
        auto adjacent_faces = new_vertex->adjacent_faces();
        for(auto f : adjacent_faces) {
            double d = -dot(f->normal(), f->halfedge()->vertex()->pos);
            Vec4 v = Vec4(f->normal(), d);
            face_quadrics[f] = outer(v, v);
            q += face_quadrics[f];
        }
        vertex_quadrics[new_vertex] = q;


        auto adjacent_vertices = new_vertex->adjacent_vertices();
        for(auto v : adjacent_vertices) {
            Mat4 q = Mat4::Zero;
            auto adjacent_faces = v->adjacent_faces();
            for(auto f : adjacent_faces) {
                q += face_quadrics[f];
            }
            vertex_quadrics[v] = q;
        }

        adjacent_edges = new_vertex->adjacent_edges();
        for(auto e : adjacent_edges) {
            Edge_Record er = Edge_Record(vertex_quadrics, e);
            edge_queue.insert(er);
            edge_records[e] = er;
        }
    }
    return true;
}
