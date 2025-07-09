- mess_type: "crumbs"
  display_name: "Crumbs"
  description: "Small, loose food debris (e.g., cereal, chips, breadcrumbs) scattered on seats and floor mats."
  tool: "vacuum"
  density_levels: ["low", "medium", "high"]
  severity: "low"
  search_tags:
    - "cereal crumbs"
    - "bread crumbs"
    - "chips crumbs"
  annotation_guidance: |
    - Draw bounding boxes around clusters of crumbs (> 5 individual pieces).
    - Tag areas with single large crumbs as low-density.
  sample_count_target: 200

- mess_type: "food_wrappers"
  display_name: "Food Wrappers"
  description: "Discarded plastic or paper packaging (e.g., candy, chip bags, sandwich wrappers)."
  tool: "pick"
  density_levels: ["single", "clustered"]
  severity: "medium"
  search_tags:
    - "food wrapper"
    - "candy wrapper"
    - "chip bag"
  annotation_guidance: |
    - Each wrapper is one instance.
    - For torn or crumpled wrappers, use polygon masks.
  sample_count_target: 150

- mess_type: "cups"
  display_name: "Cups & Bottles"
  description: "Disposable or reusable beverage containers (paper cups, plastic bottles) left behind."
  tool: "pick"
  density_levels: ["single", "multiple"]
  severity: "medium"
  search_tags:
    - "paper cup"
    - "plastic bottle"
    - "coffee cup"
  annotation_guidance: |
    - Use bounding boxes with class "cup".
  sample_count_target: 100



- mess_type: "dirt_sand"
  display_name: "Dirt & Sand"
  description: "Fine particulate debris (sand, soil dust) tracked in on shoes or objects."
  tool: "vacuum"
  density_levels: ["light", "moderate", "heavy"]
  severity: "medium"
  search_tags:
    - "sand on car seat"
    - "dirt inside car"
  annotation_guidance: |
    - Group patches of sand as regions.
    - Use bounding boxes for heavy patches.
  sample_count_target: 150

- mess_type: "paper"
  display_name: "Loose Paper"
  description: "Sheets, receipts, napkins, or paperwork scattered in the cabin."
  tool: "pick"
  density_levels: ["single", "stacked"]
  severity: "low"
  search_tags:
    - "receipt"
    - "paper sheets"
  annotation_guidance: |
    - Each paper item is a separate bounding box.
  sample_count_target: 100

- mess_type: "sandwich_crusts"
  display_name: "Sandwich Crusts"
  description: "Larger, solid food scraps (e.g., pizza crusts, sandwich edges)."
  tool: "pick"
  density_levels: ["single", "few"]
  severity: "medium"
  search_tags:
    - "sandwich crust"
    - "pizza crust"
  annotation_guidance: |
    - Use polygon masks when irregular shape.
  sample_count_target: 80

- mess_type: "hair_clumps"
  display_name: "Hair Clumps"
  description: "Clumps of human or pet hair accumulated on seats or floor mats."
  tool: "pick"
  density_levels: ["small", "large"]
  severity: "low"
  search_tags:
    - "hair clump"
    - "pet hair"
  annotation_guidance: |
    - Use polygon masks for spread-out hair.
  sample_count_target: 80

- mess_type: "crumbly_snack"
  display_name: "Snack Pieces"
  description: "Brittle snack fragments like pretzel bits, cookie crumbs."
  tool: "vacuum"
  density_levels: ["low", "medium"]
  severity: "low"
  search_tags:
    - "pretzel crumbs"
    - "cookie pieces"
  annotation_guidance: |
    - Similar to crumbs, treat clusters as single instances.
  sample_count_target: 100

- mess_type: "misc_objects"
  display_name: "Miscellaneous Objects"
  description: "Any other items (books, toys, electronics) left in cabin."
  tool: "pick"
  density_levels: ["single", "multiple"]
  severity: "low"
  search_tags:
    - "book in car"
    - "phone accessories"
  annotation_guidance: |
    - Generic "object" bounding box for non-food debris.
  sample_count_target: 50