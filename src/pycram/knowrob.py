from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
from knowrob_refills.knowrob_wrapper import KnowRob
from rosprolog_client import Prolog, PrologException
import rospy

prolog = Prolog()

def startup_knowrob():
    db = '~/workspace/whole_store_filled_neem/roslog'
    knowrob = KnowRob(initial_mongo_db=db,
                      clear_roslog=True,
                      republish_tf=True,
                      neem_mode=False)
    # knowrob = KnowRob(initial_mongo_db=None,
    #                   clear_roslog=False)
    shelf_ids = knowrob.get_shelf_system_ids(False)
    print(shelf_ids)
    for shelf_id in shelf_ids:
        print('shelf center frame id {}'.format(knowrob.get_object_frame_id(shelf_id)))
        print('shelf corner frame id {}'.format(knowrob.get_perceived_frame_id(shelf_id)))
    return knowrob

def get_all_shelves():
    #prolog = Prolog()
    return prolog.once("get_all_shelves(A)")['A']

def find_shelf_pose(shelf):
    #prolog = Prolog()
    pose = prolog.once(f"get_pose_in_desired_reference_frame('{shelf}', 'map', T, R)")
    return [pose['T'], pose['R']]

def get_pose_for_product_type(product_type):

    query = prolog.once(f"subclass_of(Product, '{product_type}'), has_type(Item, Product).")
    prolog.once("use_module(library('semweb/sparql_client')).")
    #?product tax:has_EAN ?EAN.
    sparq = " sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \
    PREFIX loc: <http://knowrob.org/kg/ProductToShelf.owl#> \
    PREFIX owl: <http://www.w3.org/2002/07/owl#> \
    PREFIX tax: <http://knowrob.org/kg/ProductTaxonomy.owl#> \
    select ?shelf{ \
        ?product rdf:type tax:TYPE. \
        ?product owl:sameAs ?prod.\
        ?prod loc:stored_in ?shelf.\
    } ', Row,\
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/TrustNonFoodKG/services/TrustNonFoodKG/sparql/')])."
    #sparq = sparq.replace('TYPE', product_type)
    #product_type = prolog.once(sparq)
    #print(product_type)
    #shelf_pose = prolog
    #query = prolog.once(f"instance_of(Product, '{product_type}'), subclass_of(Product, Class).")
    query = prolog.once(f"get_product_location('{product_type}', Item, Shelf, Layer, Facing)")
    #print(query)
    item = query['Item']
    item_pose = prolog.once(f"get_pose_in_desired_reference_frame('{item}', 'map', T, R)")
    print(item_pose)
    return [item_pose['T'], item_pose['R']]
