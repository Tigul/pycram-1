from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
from knowrob_refills.knowrob_wrapper import KnowRob
from rosprolog_client import Prolog, PrologException
import rospy
import numpy as np

prolog = Prolog()

def get_pose_for_product_type(product_type):
    prolog.once("use_module(library('semweb/sparql_client')).")
    # query = prolog.once(f"subclass_of(Product, '{product_type}'), has_type(Item, Product).")
    # prolog.once("use_module(library('semweb/sparql_client')).")
    #?product tax:has_EAN ?EAN.
    sparq = "sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \
    PREFIX loc: <http://knowrob.org/kg/ProductToShelf.owl#> \
    PREFIX owl: <http://www.w3.org/2002/07/owl#> \
    PREFIX tax: <http://knowrob.org/kg/ProductTaxonomy.owl#> \
    select ?shelf{ \
        ?product rdf:type tax:TYPE. \
        ?product owl:sameAs ?prod.\
        ?prod loc:stored_in ?shelf.\
    } ', Row,\
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/TrustNonFoodKG/services/TrustNonFoodKG/sparql/')])."
    sparq = sparq.replace('TYPE', product_type)
    product_query = prolog.once(sparq)
    print(product_query)
    if product_query == []:
        raise PrologException(f"{product_type} is not in the product taxonomy")

    # The shelf returned by the taxonomy, this is not the same as the one in the belief state
    shelf_sparq = product_query['Row']['term'][1]
    shelf_number = int(shelf_sparq.split('Shelf')[2])
    # All shelves in the belief state
    shelves = prolog.once("get_all_shelves(S)")['S']
    for shelf in shelves:
        # The shelf id tag and corresponding marker. The id tag is identical to the number
        # returned by the taxonomy
        tag = prolog.once(f"shelf_with_marker('{shelf}', Marker), holds(Marker, dmshop:markerId, ID).")
        id = int(tag['ID'].split('_')[1])
        id_shelf = np.ceil(id/2)
        if shelf_number == id_shelf:
            pose = prolog.once(f"is_at('{tag['Marker']}', ['map', T, R])")
            return pose['T'], pose['R']
    raise PrologException("No shelf position was found for the shelf the product is stored in.")


def _generate_query(args):
    query = "sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \
    PREFIX loc: <http://knowrob.org/kg/ProductToShelf.owl#> \
    ADD_PREFIX \
    PREFIX owl: <http://www.w3.org/2002/07/owl#> \
    PREFIX tax: <http://knowrob.org/kg/ProductTaxonomy.owl#> \
    select ?shelf where{ \
        ADD_QUERY \
        FILTER \
        ?product rdf:type tax:TYPE. \
        ?product owl:sameAs ?prod.\
        ?prod loc:stored_in ?shelf.\
    } ', Row,\
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/TrustNonFoodKG/services/TrustNonFoodKG/sparql/')])."

    ingredient_query = "?prod in:has_ingredient ?ingredient. \
                        ?ingredient rdf:type ?inclass."
    ingredient_filter = "FILTER(?inclass = in:ING)"
    ingredient_prefix = "PREFIX in: <http://knowrob.org/kg/ingredients.owl#>"

    query = query.replace("TYPE", "tooth_paste").replace("ADD_QUERY", ingredient_query).replace("FILTER", ingredient_filter.replace("ING", "Plant_extract")).replace("ADD_PREFIX", ingredient_prefix)
    return query
