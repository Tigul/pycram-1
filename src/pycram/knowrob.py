from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
#from knowrob_refills.knowrob_wrapper import KnowRob
from rosprolog_client import Prolog, PrologException
from types import LambdaType
import rospy
import numpy as np

prolog = Prolog()

def get_pose_for_product_type(args):
    # Load sparql client to ensure it is loaded when sending the actual query
    prolog.once("use_module(library('semweb/sparql_client')).")

    sparq = Query(args)
    product_query = prolog.once(str(sparq))
    if product_query == []:
        raise PrologException(f"{product_type} is not in the product taxonomy")

    # The shelf returned by the taxonomy, this is not the same as the one in the belief state
    shelf_sparq = product_query['Row']['term'][1]
    shelf_floor = product_query['Row']['term'][2].split('Shelffloor')[1]
    floor_height = 0.2 + int(shelf_floor) * 0.25
    shelf_number = int(shelf_sparq.split('Shelf')[1])
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
            pose['T'][2] = floor_height
            return pose['T'], pose['R']
    raise PrologException("No shelf position was found for the shelf the product is stored in.")

def get_shelf_floor_for_product(args):
    # Load sparql client to ensure it is loaded when sending the actual query
    prolog.once("use_module(library('semweb/sparql_client')).")

    sparq = Query(args)
    print(sparq)
    product_query = prolog.once(str(sparq))
    #print(product_query)
    if product_query == []:
        raise PrologException(f"{product_type} is not in the product taxonomy")

    # The shelf returned by the taxonomy, this is not the same as the one in the belief state
    shelf = product_query['Row']['term'][1].split('Shelf')[1]

    shelf_floor = product_query['Row']['term'][2].split('Shelffloor')[1]
    return shelf, shelf_floor



class Query:
    """
    This class generates a SPARQL query which asks for the shelf- and shelf floor
    location of a product corresponding to a dictionary of arguments. The arguments
    have to contain the type of the product. Additional options could be the price,
    special ingredients or the brand.

    These are the placeholder used in the base_query, every recipe must contain
    every placeholder.

    ADD_QUERY: Additional Queries that are part of the SPARQL query
    ADD_PREFIX: Additional Prefixes, these are defined before the actual query
    FILTER: Optional filters that restrict the solutions of the query
    FILL_IN: Additional placeholder in other parts that should be filled in a later step
    ADD_VAR: Additional variables that should be returned
    ORDER: How the return of the query should be ordered
    """
    #?product owl:sameAs ?instance. \n \

    base_query = "sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n \
    PREFIX loc: <http://purl.org/NonFoodKG/location#> \n \
    ADD_PREFIX \n \
    PREFIX owl: <http://www.w3.org/2002/07/owl#> \n \
    PREFIX tax: <http://purl.org/NonFoodKG/product-taxonomy#> \n \
    select ?shelf ?shelf_floor ADD_VAR where{ \n \
        ADD_QUERY \n \
        FILTER \n \
        ?product rdf:type tax:TYPE. \n \
        ?product loc:stored_in ?shelf. \n \
        ?product loc:stored_on ?shelf_floor. \n \
    } ORDER', Row, \n \
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/FoodToNonFoodKG/services/FoodToNonFood/sparql')])."

    ingredient_query = "?product in:has_ingredient in:ING."
                        #?ingredient rdf:type ?inclass."
    ingredient_filter = "FILTER(?inclass = in:ING) "
    ingredient_prefix = "PREFIX in: <http://purl.org/NonFoodKG/nonfoodingredient#> \n "

    price_query = "?product loc:has_price ?price. \n "

    brand_query = "?product gr:hasBrand brand:BRA. \n"
    store_brand_filter = "{?brandprod gr:hasBrand ?brand.} \n \
                        UNION \n \
                        {brand:BRA brand:StoreBrand ?brand.}"
    brand_prefix = "PREFIX brand: <http://purl.org/NonFoodKG/brandinfo#> \n "
    good_relations_prefix = "PREFIX gr: <http://purl.org/goodrelations/v1#>\n "

    label_prefix = "PREFIX lbl: <http://purl.org/NonFoodKG/label#> \n"
    label_query = "?product lbl:has_label lbl:LBL."

    recipes = {'ingredients' : {'ADD_QUERY': ingredient_query,
                                'FILTER': '',
                                'ADD_PREFIX': ingredient_prefix,
                                'FILL_IN': 'ING',
                                'ADD_VAR': '',
                                'ORDER': ''},
                'price' : {'ADD_QUERY': price_query,
                            'FILTER' : '',
                            'ADD_PREFIX' : '',
                            'ADD_VAR': '?price',
                            'ORDER': lambda args: 'ORDER BY ?price' if args['price'] == 'cheapest' else ('ORDER BY DESC(?price)' if args['price'] == 'priciest' else '') },
                'brand' : {'ADD_QUERY' : brand_query,
                            #'FILTER' : store_brand_filter,
                            'FILTER': '',
                            'ADD_PREFIX' : brand_prefix + " " + good_relations_prefix,
                            'FILL_IN': 'BRA',
                            'ADD_VAR': '',
                            'ORDER': ''},
                'label': {'ADD_QUERY' : label_query,
                            'FILTER': '',
                            'ADD_PREFIX' : label_prefix,
                            'FILL_IN': 'LBL',
                            'ADD_VAR': '',
                            'ORDER': ''}}

    def __init__(self, args):
        """
        The constructor takes the arguments and calls the respective methods to
        generate the Query.
        :param args: A dictionary of properties the product, for which this query
        asks, should have.
        """
        self.fill_in = {'TYPE' : args['product']}
        self.args = args
        self.generated_query = self._generate_query(args)
        self.filled_query = ""
        self._fill_query()

    def _generate_query(self, args):
        """
        Checks which recipes are needed for this query, the strings of the
        used recipes are then combined and replace the placeholder in the base query.
        """
        working_list = []
        for key in args.keys():
            if key in Query.recipes.keys():
                # Check if new placeholder are defined
                if 'FILL_IN' in Query.recipes[key].keys():
                    self.fill_in[Query.recipes[key]['FILL_IN']] = args[key]
                working_list.append(Query.recipes[key])
        working_list = self._combine_dicts(working_list)
        res = Query.base_query
        # Build the final query
        for key, value in working_list.items():
            res = res.replace(key, value)
        return res

    def _combine_dicts(self, dicts):
        """
        Combines the strings for the same placeholder for all recipes that are
        used in this query.
        """
        res_dict = {}
        for key in ['ADD_QUERY', 'FILTER', 'ADD_PREFIX', 'ADD_VAR', 'ORDER']:
            res_dict[key] = ""
            for d in dicts:
                # If the value is of type lambda call it and then insert the return
                if type(d[key]) == LambdaType:
                    res_dict[key] += d[key](self.args) + " "
                # If the value has any other type append it to the resulting dictionary
                else:
                    res_dict[key] += d[key] + " "
        return res_dict


    def _fill_query(self):
        """
        Fills the remaining placeholder which are specific for a specific product.
        Here all placeholder created by additional queries are inserted.
        The inserted values could be, for example, the product type or brand name.
        """
        self.filled_query = self.generated_query
        for key, value in self.fill_in.items():
            self.filled_query = self.filled_query.replace(key, value)

    def __str__(self):
        return self.filled_query
    def __call__(self):
        return self.filled_query
