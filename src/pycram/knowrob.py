from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
from knowrob_refills.knowrob_wrapper import KnowRob
from rosprolog_client import Prolog, PrologException
from types import LambdaType
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
    select ?shelf ?product ?shelf_floor ?price { \
        ?product rdf:type tax:TYPE. \
        ?product owl:sameAs ?prod.\
        ?prod loc:stored_in ?shelf.\
        ?prod loc:stored_on ?shelf_floor. \
        ?prod loc:has_price ?price. \
    } ORDER BY ?price', Row,\
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/TrustNonFoodKG/services/TrustNonFoodKG/sparql/')])."
    sparq = sparq.replace('TYPE', product_type)
    #sparq = _generate_query({})
    #
    # sparq = "sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \
    # PREFIX loc: <http://knowrob.org/kg/ProductToShelf.owl#> \
    # PREFIX in: <http://knowrob.org/kg/ingredients.owl>\
    # PREFIX owl: <http://www.w3.org/2002/07/owl#> \
    # PREFIX tax: <http://knowrob.org/kg/ProductTaxonomy.owl#> \
    # select ?shelf where{ \
    #     ?prod in:has_ingredient ?ingredient. \
    #     ?ingredient rdf:type ?inclass. \
    #     FILTER(?inclass = in:Plant_extract) \
    #     ?prod owl:sameAs ?product. \
    #     ?product rdf:type tax:toothpaste. \
    #     ?product loc:stored_in ?shelf.\
    # } ', Row,\
    # [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/TrustNonFoodKG/services/TrustNonFoodKG/sparql/')])."


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
    base_query = "sparql_query('PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n \
    PREFIX loc: <http://knowrob.org/kg/location.owl#> \n \
    ADD_PREFIX \n \
    PREFIX owl: <http://www.w3.org/2002/07/owl#> \n \
    PREFIX tax: <http://knowrob.org/kb/product-taxonomy.owl#> \n \
    select ?shelf ?shelf_floor ADD_VAR where{ \n \
        ADD_QUERY \n \
        FILTER \n \
        ?product rdf:type tax:TYPE. \n \
        ?product owl:sameAs ?instance. \n \
        ?instance loc:stored_in ?shelf. \n \
        ?instance loc:stored_on ?shelf_floor. \n \
    } ORDER', Row, \n \
    [ endpoint('https://api.krr.triply.cc/datasets/mkumpel/ProductCatalog/services/FoodToNonFood/sparql')])."

    ingredient_query = "?prod in:has_ingredient ?ingredient. \n \
                        ?ingredient rdf:type ?inclass. \n \
                        ?prod owl:sameAs ?product."
    ingredient_filter = "FILTER(?inclass = in:ING) "
    ingredient_prefix = "PREFIX in: <http://knowrob.org/kg/ingredients.owl#> \n "

    price_query = "?instance loc:has_price ?price. \n "

    store_brand_filter = "{?brandprod gr:hasBrand ?brand.} \n \
                        UNION \n \
                        {brand:BRA brand:StoreBrand ?brand.}"
    brand_prefix = "PREFIX brand: <http://knowrob.org/kg/brandinfo.owl#> \n "
    good_relations_prefix = "PREFIX gr: <http://purl.org/goodrelations/v1#>\n "

    recipes = {'ingredients' : {'ADD_QUERY': ingredient_query,
                                'FILTER': ingredient_filter,
                                'ADD_PREFIX': ingredient_prefix,
                                'FILL_IN': 'ING',
                                'ADD_VAR': '',
                                'ORDER': ''},
                'price' : {'ADD_QUERY': price_query,
                            'FILTER' : '',
                            'ADD_PREFIX' : '',
                            'ADD_VAR': '?price',
                            'ORDER': lambda args: 'ORDER BY ?price' if args['price'] == 'cheapest' else ('ORDER BY DESC(?price)' if args['price'] == 'highest' else '') },
                'brand' : {'ADD_QUERY' : '',
                            'FILTER' : store_brand_filter,
                            'ADD_PREFIX' : brand_prefix + " " + good_relations_prefix,
                            'FILL_IN': 'BRA',
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
                if 'FILL_IN' in Query.recipes[key].keys():
                    self.fill_in[Query.recipes[key]['FILL_IN']] = args[key]
                working_list.append(Query.recipes[key])
        working_list = self._combine_dicts(working_list)
        res = Query.base_query
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
                if type(d[key]) == LambdaType:
                    res_dict[key] += d[key](self.args) + " "
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
