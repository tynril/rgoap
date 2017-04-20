#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate pathfinding;

use std::collections::BTreeMap;
use std::hash::{Hash, Hasher};
use pathfinding::astar;

/// A map of state atoms to their values.
type State = BTreeMap<String, bool>;

/// An action that can be used to influence the world state.
#[derive(Serialize, Deserialize, PartialEq, Eq)]
pub struct Action {
    name: String,
    cost: usize,
    pre_conditions: State,
    post_conditions: State,
}

/// A node in the planner graph.
#[derive(PartialEq, Eq, Clone)]
struct PlanNode<'a> {
    current_state: State,
    action: Option<&'a Action>,
}

impl<'a> Hash for PlanNode<'a> {
    fn hash<H>(&self, state: &mut H)
        where H: Hasher
    {
        if let Some(action) = self.action {
            action.name.hash(state);
        }

        for (key, value) in &self.current_state {
            key.hash(state);
            value.hash(state);
        }
    }
}

impl<'a> PlanNode<'a> {
    /// Makes an initial plan node without a parent.
    fn initial(initial_state: &'a State) -> PlanNode<'a> {
        PlanNode {
            current_state: initial_state.clone(),
            action: None,
        }
    }

    /// Makes a plan node from a parent state and an action applied to that state.
    fn child(parent_state: State, action: &'a Action) -> PlanNode<'a> {
        let mut child = PlanNode {
            current_state: parent_state.clone(),
            action: Some(action),
        };

        // Applies the post-condition of the action applied on our parent state.
        for (name, value) in &action.post_conditions {
            child.current_state.insert(name.clone(), value.clone());
        }

        child
    }

    /// Returns all possible nodes from this current state, along with the cost to get there.
    fn possible_next_nodes(&self, actions: &'a [Action]) -> Vec<(PlanNode<'a>, usize)> {
        let mut nodes: Vec<(PlanNode<'a>, usize)> = vec![];
        for action in actions {
            if self.matches(&action.pre_conditions) {
                nodes.push((PlanNode::child(self.current_state.clone(), action), action.cost));
            }
        }

        nodes
    }

    /// Count the number of states in this node that aren't matching the given target.
    fn mismatch_count(&self, target: &State) -> usize {
        let mut count: usize = 0;
        for (name, target_value) in target {
            if let Some(current_value) = self.current_state.get(name) {
                if current_value != target_value {
                    count += 1;
                }
            } else {
                count += 1;
            }
        }

        count
    }

    /// Returns `true` if the current node is a full match for the given target.
    fn matches(&self, target: &State) -> bool {
        self.mismatch_count(target) == 0
    }
}

/// Formulates a plan to get from an initial state to a goal state using a set of allowed actions.
pub fn plan<'a>(initial_state: &'a State,
                goal_state: &State,
                allowed_actions: &'a [Action])
                -> Option<Vec<&'a Action>> {
    // Builds our initial plan node.
    let start = PlanNode::initial(initial_state);

    // Runs our search over the states graph.
    if let Some((plan, _)) = astar(&start,
                                   |ref node| node.possible_next_nodes(allowed_actions),
                                   |ref node| node.mismatch_count(goal_state),
                                   |ref node| node.matches(goal_state)) {
        Some(plan.into_iter().skip(1).map(|ref node| node.action.unwrap()).collect())
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    extern crate serde_json;

    use super::*;
    use std::path::Path;
    use std::fs;

    /// A test case
    #[derive(Deserialize)]
    struct TestCase {
        #[serde(skip_deserializing)]
        case_name: String,

        actions: Vec<Action>,
        initial_state: State,
        goal_state: State,
        expected_actions: Vec<String>,
    }

    impl TestCase {
        /// Loads a test case from a JSON file.
        fn from_case_file(path: &Path) -> TestCase {
            let file = fs::File::open(path).unwrap();
            let mut case: TestCase = serde_json::from_reader(file).unwrap();
            case.case_name = String::from(path.file_name().unwrap().to_str().unwrap());
            case
        }

        /// Checks if the computed plan matches the expectation.
        fn assert_plan(&self) {
            let plan = plan(&self.initial_state, &self.goal_state, &self.actions);

            if let Some(actions_list) = plan {
                let actions_names: Vec<String> =
                    actions_list.iter().map(|&action| action.name.clone()).collect();
                if self.expected_actions != actions_names {
                    panic!("{} failed: expected {:?}, got {:?}",
                           self.case_name,
                           self.expected_actions,
                           actions_names);
                }
            } else {
                if self.expected_actions.len() > 0 {
                    panic!("{} failed: expected {:?}, got no plan",
                           self.case_name,
                           self.expected_actions);
                }
            }
        }
    }

    #[test]
    fn run_all_test_cases() {
        let paths = fs::read_dir("./data").unwrap();
        for path in paths {
            let case = TestCase::from_case_file(path.unwrap().path().as_path());
            case.assert_plan();
        }
    }
}
